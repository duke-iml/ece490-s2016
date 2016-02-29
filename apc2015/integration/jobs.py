import logging, traceback
logger = logging.getLogger(__name__)

from multiprocessing import Process, Pipe

class BaseJob:
    def __init__(self, init_args, method, args=None, kwargs=None):
        self.clear()

        self.init_args = init_args
        self.method = method
        self.args = args or ()
        self.kwargs = kwargs or {}

    # TODO: need to understand this method. This is the key to spawning and running processes
    def run(self):
        # create a pipe to communicate with the child
        (self.pipe, self.child_pipe) = Pipe()

        # create the subprocess
        self.child = Process(target=self._handler)
        # set the child to run as a background process (i.e., exit when parent does)
        self.child.daemon = True

        self.child.start()

    def _get_interface(self):
        raise NotImplemented

    # this runs in the remote process
    def _handler(self):
        logger.debug('job {} start: {} {} {}'.format(self.child.pid, self.method, self.args, self.kwargs))

        try:
            # construct the interface --> will get NotImplemented exception
            interface = self._get_interface()(*self.init_args)

            # call the target method
            result = getattr(interface, self.method)(*self.args, **self.kwargs)
            logger.info('job {} done: {}'.format(self.child.pid, result))
        except Exception as e:
            # send the error
            self.child_pipe.send(e)
            logger.error('job {} error: {}'.format(self.child.pid, e))
            logger.error(traceback.format_exc())
        else:
            # return the result
            self._send_result(result)

        self._close()

    def wait(self, timeout=None):
        self.child.join(timeout)

    def cancel(self):
        # try to avoid this
        self.child.terminate()
        # this join is needed to avoid zombie processes
        self.child.join()

        self._done = True
        self._error = Exception('early cancellation')

    def clear(self):
        self._done = False
        self._result = None
        self._error = None

    def _send_result(self, result):
        self.child_pipe.send(result)

    def _process_result(self):
        # there is a result to read
        try:
            # protect against IO errors when reading the pipe
            output = self.pipe.recv()
        except Exception as e:
            logger.error('error reading result from job {}:'.format(self.child.pid, e))
            logger.error(traceback.format_exc())
            output = e
        # distinguish between actual results and errors
        if isinstance(output, Exception):
            self._error = output
        else:
            self._result = output

    def _check(self):
        # check if the child finished
        if self.alive:
            return

        # check for a result before attempting a blocking call
        if self.pipe.poll():
            self._process_result()
        else:
            # for some reason the child did not send an output
            logger.warn('job {} did not send result (died or killed?)'.format(self.child.pid))
            self._error = Exception('no result sent')

        self._close()
        self._done = True

    def _close(self):
        # clean up
        self.pipe.close()
        self.child_pipe.close()

    @property
    def done(self):
        if not self._done:
            self._check()

        return self._done

    @property
    def result(self):
        if not self._done:
            self._check()

        return self._result

    @property
    def error(self):
        if not self._done:
            self._check()

        return self._error

    @property
    def alive(self):
        return self.child.is_alive()

class JobManager:
    def __init__(self, factory=None, jobs=None, count=10):
        if jobs:
            self.count = len(jobs)
            self.jobs = jobs
        else:
            self.count = count
            self.jobs = [ factory() for i in range(count) ]

    def run(self):
        for job in self.jobs:
            job.run()

    def all_done(self):
        return self.count_done() == self.count

    def count_done(self):
        return len(self.get_done())

    def count_success(self):
        return len(self.get_success())

    def count_error(self):
        return len(self.get_error())

    def get_success(self):
        return filter(lambda job: job.result is not None and not job.error, self.jobs)

    def get_done(self):
        return filter(lambda job: job.done, self.jobs)

    def get_error(self):
        return filter(lambda job: job.error is not None, self.jobs)

    def cancel(self):
        for job in self.jobs:
            if not job.done:
                job.cancel()

class JobServer:
    def __init__(self, init_args, methods, factory, parallelism=10):
        self.init_args = init_args
        self.factory = factory
        self.parallelism = parallelism

        # set up the interface
        for method in methods:
            # this outher wrapping function is necessary to capture the method variable
            def make_proxy(method):
                def proxy(*args, **kwargs):
                    return self._call(method, args, kwargs)
                return proxy

            setattr(self, method, make_proxy(method))

    def _call(self, method, args, kwargs):
        manager = JobManager(
            # factory = function handle of PlanningJob(knowledgeBase, method, args, kwargs)
            factory=lambda: self.factory(self.init_args, method, args, kwargs),
            count=self.parallelism
        )

        manager.run()
        return manager
