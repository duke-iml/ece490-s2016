"""Implements a notion of "topics" as members of a StructureService.
Using topics on a single service to communicate data is often easier
than directly sending/receiving data from multiple services.  There is
an overhead in this method in that the structure server must read the
data and then re-send it to any subscribers.
"""

from service import Service
from structure_service import StructureService
import time
import asyncore
import socket

class TopicServiceBase (Service):
    """A base class for a service that accesses a topic on a StructureService.
    Accepts name, subscribe, unsubscribe, set, get, change, delete, and
    resize."""
    def __init__(self,topic='.'):
        self.topic = topic
        Service.__init__(self)

    def open(self,addr):
        Service.open(self,addr,asServer=False)

    def setName(self,name):
        """Helper function: may be used by subclasses / callers"""
        self.sendMessage({'type':'name','data':name})

    def subscribe(self,rate=0):
        """Helper function: may be used by subclasses / callers"""
        self.sendMessage({'type':'subscribe','path':self.topic,'rate':rate})

    def unsubscribe(self=0):
        """Helper function: may be used by subclasses / callers"""
        self.sendMessage({'type':'unsubscribe','path':self.topic})

    def set(self,value):
        """Helper function: may be used by subclasses / callers"""
        self.sendMessage({'type':'set','path':self.topic,'data':value})

    def get(self):
        """Helper function: may be used by subclasses / callers"""
        self.sendMessage({'type':'get','path':self.topic})

    def change(self,value):
        """Helper function: may be used by subclasses / callers"""
        self.sendMessage({'type':'change','path':self.topic,'data':value})

    def delete(self):
        """Helper function: may be used by subclasses / callers"""
        self.sendMessage({'type':'delete','path':self.topic})

    def resize(self,n):
        """Helper function: may be used by subclasses / callers"""
        self.sendMessage({'type':'resize','path':self.topic,'data':n})

    def onMessage(self,msg):
        """Overload this to handle responses to subscribe() and get()"""
        if msg == "SSPP StructureService": pass
        pass

class TopicSubscriberBase(TopicServiceBase):
    """Automatically subscribes to a topic on the given address.
    subscriber = TopicSubscriberBase(addr,topic,rate).
    
    Subclass still must overload onMessage()
    """
    def __init__(self,addr,topic,rate=0):
        TopicServiceBase.__init__(self,topic)
        print "TopicSubscriberBase",self.topic,"initializing"
        TopicServiceBase.open(self,addr)
        print "TopicSubscriberBase",self.topic,"subscribing"
        self.subscribe(rate)

class TopicEchoService (TopicSubscriberBase):
    """An example class for how to subclass TopicSubscriberBase."""
    def onMessage(self,msg):
        print "Received message:",msg

class TopicLogService (TopicSubscriberBase):
    """Dumps a log to disk."""
    def __init__(self,logfn,addr,topic='.'):
        TopicSubscriberBase.__init__(self,addr,topic)
        self.sendMessage({'type':'name','data':'Topic Log Service'})
        self.fout = open(logfn,'w')
    def onMessage(self,msg):
        fout.write(json.dumps(msg))
        fout.write('\n')
    def handle_close(self):
        self.fout.close()

class TopicValue (TopicServiceBase):
    """This class allows you to easily get and set a value on a
    StructureService. The value may be changed by external processes, and the
    stored value will be automatically updated (thread safe).

    Usage is
    v = TopicValue(addr=('localhost',4567),topic='.foo')
    v.set({'bar':[4,5,6]})
    print v.get()
    """
    def __init__(self,addr,topic='.'):
        TopicServiceBase.__init__(self,topic)
        try:
            self.open(addr)
        except Exception as e:
            print "Couldn't open..."            
        self.setName('TopicValue('+topic+')')
    def get(self,timeout=None):
        TopicServiceBase.get(self)
        return self.waitForMessage(timeout)

class TopicListener (TopicSubscriberBase):
    """This class allows you to easily get and set a value on a
    StructureService. The value may be changed by external processes, and the
    stored value will be automatically updated (thread safe).

    Usage is
    v = TopicListener(addr=('localhost',4567),topic='.foo')
    v.set({'bar':[4,5,6]})
    print v.get()

    The main difference between TopicListener and TopicValue is that
    this class will not block on get().  As a result it may be
    somewhat faster if gets are performed frequently while the
    structure stored by the StructureService changes infrequently.
    """
    def __init__(self,addr,topic='.',rate=0):
        self.value = None
        TopicSubscriberBase.__init__(self,addr,topic)
        self.setName('TopicListener('+topic+')')
    def get(self):
        return self.value
    def set(self,value):
        self.sendMessage({'type':'set','path':self.topic,'data':value})
        self.value = value
    def onMessage(self,msg):
        #overload of Service method
        #print "TopicListener",self.topic,"got message",msg
        self.value = msg

class TopicTransformService (TopicSubscriberBase):
    """A service that will subscribe to a topic, transform it, and publish it
    back to a different topic.
    """
    def __init__(self,func,addr,sourceTopic,destTopic):
        self.func = func
        self.lastSentMsg = None
        self.sourceTopic = sourceTopic
        self.destTopic = destTopic
        self.checkLoop = (sourceTopic == destTopic)
        TopicSubscriberBase.__init__(self,addr,sourceTopic)
        self.setName('TopicListener('+sourceTopic+','+destTopic+')')
    def onMessage(self,msg):
        if self.checkLoop:
            if msg == self.lastSentMsg:
                #avoid infinite loops if sourceTopic == destTopic
                return
        #process and send
        res = self.func(msg)
        self.sendMessage({'type':'set','path':self.destTopic,'data':res})
        if self.checkLoop:
            self.lastSentMsg = msg

class TopicServer(StructureService):
    """A convenience class that runs a topic server on the initialized
    address."""
    def __init__(self,addr=None):
        StructureService.__init__(self)
        if addr:
            self.open(addr,asServer=True)
    def open(self,addr):
        StructureService.open(self,addr,asServer=True)

if __name__ == '__main__':
    service = TopicEchoService(('localhost',4568),'.')
    service.run()
