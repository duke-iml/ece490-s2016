#ifndef SSPP_TOPIC_H
#define SSPP_TOPIC_H

#include "Service.h"

namespace SSPP {
  using namespace std;

class TopicServiceBase : public Service
{
 public:
  string topic;

  TopicServiceBase(const char* host="tcp://localhost:4567",const char* topic=".",double timeout=0);
  virtual ~TopicServiceBase() {}
  virtual const char* Name() const { return "TopicServiceBase"; }
  virtual const char* Description() const { return "Does something with a topic server"; }
  void Subscribe(double rate=0);
  void Set(const AnyCollection& data);
  void Get();
  void Change(const AnyCollection& data);
  void Resize(size_t n);
  void Delete();
  //Override this to handle messages from subscribe / get
  virtual bool OnMessage(AnyCollection& message) { return true; }
};

class TopicListener : public TopicServiceBase
{
 public:
  TopicListener(const char* host="tcp://localhost:4567",const char* topic=".",double timeout=0);
  virtual ~TopicListener() {}
  virtual const char* Name() const { return "TopicListener"; }
  virtual const char* Description() const { return "Listens to a topic on a topic server"; }
  virtual bool OnMessage(AnyCollection& message) { value = message; return true; }
  AnyCollection& Get() { return value; }

  AnyCollection value;
};

class TopicValue : public TopicServiceBase
{
 public:
  TopicValue(const char* host="tcp://localhost:4567",const char* topic=".",double timeout=0);
  virtual ~TopicValue() {}
  virtual const char* Name() const { return "TopicValue"; }
  virtual const char* Description() const { return "Reads/writes to a topic on a topic server"; }
  virtual bool OnMessage(AnyCollection& message);
  AnyCollection& Get(double timeout=Math::Inf);

  AnyCollection value;
};

///Convenience function: connects to a service and requests a topic.  Returns
///an empty collection on error.
AnyCollection ReadTopic(const char* addr,const char* topic=".",double timeout=0);

} //namespace SSPP

#endif 
