from multiprocessing import Process
import os

def info(title):
    print title
    print 'module name:', __name__
    if hasattr(os, 'getppid'):  # only available on Unix
        print 'parent process:', os.getppid()
    print 'process id:', os.getpid()

def f(name):
    info('function f')
    print 'hello', name

if __name__ == '__main__':
    info('main line')

    # http://stackoverflow.com/questions/1559125/string-arguments-in-python-multiprocessing
    # If you want to have a tuple with only one element, you need to specify that it's actually
    # a tuple (and not just something with brackets around it) - this is done by adding a comma after the element.
    p = Process(target=f, args=('bob',))
    p.start()
    p.join()
