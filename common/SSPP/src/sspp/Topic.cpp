#include <sspp/Topic.h>

namespace SSPP {

TopicServiceBase::TopicServiceBase(const char* host,const char* _topic,double timeout) 
  :topic(_topic)
{ 
  if(!OpenClient(host,timeout)) {
    printf("Waiting for host %s to open\n",host);
  }
}
void TopicServiceBase::Subscribe(double rate) 
{
  AnyCollection msg;
  msg["type"] = string("subscribe");
  msg["path"] = topic;
  if(rate!=0)
    msg["rate"] = rate;
  SendMessage(msg);
}

void TopicServiceBase::Set(const AnyCollection& data) 
{
  AnyCollection msg;
  msg["type"] = string("set");
  msg["path"] = topic;
  msg["data"] = data;
  SendMessage(msg);
}

void TopicServiceBase::Get() 
{
  AnyCollection msg;
  msg["type"] = string("get");
  msg["path"] = topic;
  SendMessage(msg);
}

void TopicServiceBase::Change(const AnyCollection& data) 
{
  AnyCollection msg;
  msg["type"] = string("change");
  msg["path"] = topic;
  msg["data"] = data;
  SendMessage(msg);
}

void TopicServiceBase::Resize(size_t n) 
{
  AnyCollection msg;
  msg["type"] = string("resize");
  msg["path"] = topic;
  msg["data"] = int(n);
  SendMessage(msg);
}

void TopicServiceBase::Delete() 
{
  AnyCollection msg;
  msg["type"] = string("delete");
  msg["path"] = topic;
  SendMessage(msg);
}

TopicListener::TopicListener(const char* host,const char* topic,double timeout)
  :TopicServiceBase(host,topic,timeout)
{
  Subscribe();
  //this is helpful for slow readers / fast senders
  onlyProcessNewest=true;
}

TopicValue::TopicValue(const char* host,const char* topic,double timeout)
    :TopicServiceBase(host,topic,timeout)
{
  //this is helpful for slow readers / fast senders
  onlyProcessNewest = true;
}

bool TopicValue::OnMessage(AnyCollection& message)
{
  value = message;
  return true;
}

AnyCollection& TopicValue::Get(double timeout)
{
  TopicServiceBase::Get();
  Timer timer;
  value.clear();
  while(value.size()==0) {
    int n = Process();
    if(n < 0) {
      printf("TopicValue: error reading while waiting for Get() response\n");
      return value;
    }
    if(value.size()!=0) return value;
    if(!Connected()) {
      printf("TopicValue: was disconnected while waiting for Get() response\n");
      return value;
    }
    if(timer.ElapsedTime() > timeout) {
      printf("TopicValue: timeout reached while waiting for Get() response\n");
      return value;
    }
    ThreadSleep(0.01);
  }
  return value;
}

class TopicReaderService : public TopicServiceBase
{
public:
  vector<AnyCollection> messages;
  int max;

  TopicReaderService(const char* addr,const char* topic,double timeout,int _max=1)
    :TopicServiceBase(addr,topic,timeout),max(_max)
  {
    Get();
  }
  virtual int Process() {
    int n=TopicServiceBase::Process();
    if((int)messages.size() >= max) return -1;
    return n;
  }
  virtual bool OnMessage(AnyCollection& m) 
  { 
    cout<<"TopicReader: message "<<m<<endl;
    messages.push_back(m); 
    if((int)messages.size() >= max) return false;
    return true; 
  }
};

AnyCollection ReadTopic(const char* addr,const char* topic,double timeout)
{
  printf("ReadTopic...\n");
  SyncPipe pipe;
  pipe.transport = new SocketClientTransport(addr);
  if(!pipe.Start()) {
    printf("ReadTopic: Couldn't connect to %s\n",addr);
    return AnyCollection();
  }
  AnyCollection msg;
  msg["type"] = string("get");
  msg["path"] = string(topic);
  stringstream ss;
  ss<<msg;
  pipe.SendMessage(ss.str());
  Timer timer;
  while(true) {
    if(timeout > 0 && timer.ElapsedTime() >= timeout) break;
    pipe.Work();
    if(pipe.NewMessageCount() > 0) {
      stringstream ss(pipe.NewestMessage());
      AnyCollection res;
      ss>>res;
      if(!ss) {
	printf("ReadTopic: incorrectly formatted reply to %s/%s\n",addr,topic);
	return AnyCollection();
      }
      //success
      return res;
    }
    //didn't receive reply yet
    printf("ReadTopic: waiting...\n");
    ThreadSleep(0.01);
  }
  printf("ReadTopic: Timeout occurred\n");
  return AnyCollection();
}


} //namespace SSPP
