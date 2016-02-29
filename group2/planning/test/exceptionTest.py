class exceptionTest:

    def _get_interface(self):
            print 1
            raise NotImplemented

    # this runs in the remote process
    def _handler(self):
        print 2
        try:
            print 3
            # construct the interface --> will get NotImplemented exception
            interface = self._get_interface()

            # call the target method
            print 4
        except Exception as e:
            # send the error
            print 5
        else:
            # return the result
            print 6

if __name__ == '__main__':

    et = exceptionTest()
    et._handler()
