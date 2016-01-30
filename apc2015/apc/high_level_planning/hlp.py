import copy
import re
from database import Database
from collections import deque
import itertools
from klampt import vectorops
import sys

re_braces = re.compile("\{\w+?\}")
re_dollar = re.compile("\$\w+")

def delta(dict1,dict2):
    """Returns the changes from dict2 to dict1 """
    res = {}
    for k,v in dict1.iteritems():
        if k not in dict2:
            res[k] = v
        elif dict2[k] != v:
            if isinstance(dict2[k],dict) and isinstance(v,dict):
                res[k] = delta(v,dict2[k])
            else:
                res[k] = v
    return res

def gen_add(a,b):
    """a and b can be floats, integers, lists of floats/integers, or None.
    If both are lists, must be of same length"""
    if a == None: return b
    if b == None: return a
    if isinstance(a,(list,tuple)):
        if isinstance(b,(list,tuple)):
            return vectorops.add(a,b)
        else:
            return [ai + b for ai in a]
    elif isinstance(b,(list,tuple)):
        return [bi + a for bi in b]
    else:
        return a+b

def gen_mul(a,b):
    """a and b can be floats, integers, lists of floats/integers, or None.
    If both are lists, must be of same length"""
    if a == None: return None
    if b == None: return None
    if isinstance(a,(list,tuple)):
        if isinstance(b,(list,tuple)):
            return [ai * bi for (ai,bi) in zip(a,b)]
        else:
            return [ai*b for ai in a]
    elif isinstance(b,(list,tuple)):
        return [bi*a for bi in b]
    else:
        return a*b


def split_array_access(string):
    """Given a string of the form 'element', 'element.subelement', or
    'element[subelement]', this will split the string into an (element,suffix)
    pair."""
    if string.find('.') >= 0:
        dpos = string.find('.')
        return string[:dpos],string[dpos:]
    if string.find('[') >= 0:
        dpos = string.find('[')
        return string[:dpos],string[dpos:]
    return string,""

def parse_expr(expr):
    """With an expression that refers to state variables in braces
    i.e., {state_var}, and argument variables with $, i.e., $arg_var,
    replaces them with _state_['state_var'] and _args_['arg_var'].
    Returns (processed_expr,state_deps,"""
    global re_braces,re_dollar
    state_deps = []
    arg_deps = []
    def f(match):
        state_deps.append(match.group()[1:-1])
        return '_state_["'+match.group()[1:-1]+'"]'
    def g(match):
        arg_deps.append(match.group()[1:])
        return '_args_["'+match.group()[1:]+'"]'
    res = re_braces.sub(f,expr)
    res = re_dollar.sub(g,res)
    return res,state_deps,arg_deps

def enumerate_combinations(domain_dict):
    keys = domain_dict.keys()
    domains = domain_dict.values()
    augmented_domains = []
    for k,d in zip(keys,domains):
        augmented_domains.append([(k,v) for v in d])
    for element in itertools.product(*augmented_domains):
        yield dict(element)
    return

class Expression:
    """Given an string expression referring to state variables in the form
    {object} and argument variables in the form $arg, allows evaluation
    of the string in the scope of a given state / argument / context.
    """
    def __init__(self,string):
        self.source_string = string
        self.parsed_expr,self.state_deps,self.arg_deps = parse_expr(string)
    def state_dependent(self):
        return len(self.state_deps) > 0
    def arg_dependent(self):
        return len(self.arg_deps) > 0
    def evaluate(self,state,args,context=None):
        """Evaluates the results of an expression on the given state with the
        given arguments.
        
        Arguments:
        - state: dict mapping names to values.
        - args: dict mapping names to values.
        - context: a module giving the context of the

        The expression is assumed to have no side effects (possible
        security warning!)"""
        loc = {'_state_':state,
               '_args_':args}
        for s in self.state_deps:
            if s not in state:
                state[s] = None
        try:
            return eval(self.parsed_expr,context.__dict__,loc)
        except KeyError as e:
            return None
        except Exception as e:
            print "Exception thrown when evaluating expression:",e
            print type(e).__name__
            print self.source_string
            raise

def parse_compound_expr(string):
    if string.split()=='':
        return []
    components = string.split(';')
    return [Expression(c) for c in components if c.strip() != '']

class CompoundCondition:
    """A semicolon-separated list of expressions that all must be true"""
    def __init__(self,string):
        self.exprs = parse_compound_expr(string)
    def evaluate(self,state,args,context=None):
        for e in self.exprs:
            if not e.evaluate(state,args,context):
                return False
        return True
    def evaluateStateOnly(self,state,context=None):
        """Returns false if any argument-independent conditions are false"""
        for e in self.exprs:
            if not e.arg_dependent():
                if not e.evaluate(state,{},context):
                    return False
        return True


class Effect:
    """Describes a set of effects from an action.  It contains a list of
    assignments of the form item_name=[python_expression] which affect
    the state.  It may also be optionally named"""
    def __init__(self,name=None,items=None,expressions=None):
        self.name = name
        if items==None: items=[]
        if expressions==None: expressions=[]
        self.items = items
        self.expressions = expressions
        self.assignment_expressions = []
        for i,e in zip(items,expressions):
            item,suffix = split_array_access(i)
            self.assignment_expressions.append(Expression('_nextstate_["'+item+'"]'+suffix+' = '+e.source_string))
        
    def parse(self,string):
        components = string.split(';')
        start = 0
        if '=' not in components[0]:
            self.name = components[0].strip()
            start += 1
        self.items = []
        self.expressions = []
        self.assignment_expressions = []
        for c in components[start:]:
            parts = c.split('=',1)
            assert len(parts)==2,"Expression "+c+" does not have form 'item = expr'"
            self.items.append(parts[0].strip())
            self.expressions.append(Expression(parts[1]))
            item,suffix = split_array_access(parts[0].strip())
            self.assignment_expressions.append(Expression('_nextstate_["'+item+'"]'+suffix+' = '+parts[1]))
        return
    
class ActionResult:
    """An action result has a condition under which it may take effect,
    and a set of effects (list of Effect objects) with associated
    probabilities."""
    def __init__(self,condition=None,effects=None,probabilities=None):
        if effects == None: effects = []
        if probabilities == None: probabilities = []
        assert len(effects)==len(probabilities)
        self.condition = condition
        self.effects = effects
        self.probabilities = probabilities

class Action:
    """
    - name: the action name
    - args: a dictionary mapping argument names to domains (e.g., a set of
      values
    - results: a list of ActionResult objects describing conditions and
      possible effects of this action"""
    def __init__(self,name=None,args=None,results=None,context=None):
        if args == None: args = {}
        if results == None: results = []

        self.name = name
        self.args = args
        self.results = results
        self.context = context

    def applicable_args(self,state):
        if len(self.results)==0: return []
        possibleresults = []
        for r in self.results:
            applicable = True
            for e in r.condition.exprs:
                if not e.arg_dependent():
                    if not e.evaluate(state,{},self.context):
                        applicable = False
                        break
            if applicable:
                possibleresults.append(r)
        if len(possibleresults)==0:
            return []
        if len(self.args)==0:
            for r in self.results:
                applicable = True
                for e in r.condition.exprs:
                    if e.arg_dependent():
                        raise ValueError("Error, arg-free action "+self.name+" depends on some $ arguments")
            return [{}]
        res = []
        for args in enumerate_combinations(self.args):
            for r in possibleresults:
                #for e in r.condition.exprs:
                #    print e.parsed_expr
                if r.condition.evaluate(state,args,self.context):
                    res.append(args)
                    break
        #no applicable conditions
        return res

    def applicable(self,state,args={}):
        assert len(args)==len(self.args)
        for k,v in args.iteritems():
            if k not in self.args[k]:
                return False
        if len(self.results)==0: return False
        for r in self.results:
            isapplicable = True
            for c in r.conditions:
                if not c.evaluate(state,args,self.context):
                    isapplicable=False
                    break
            if isapplicable:
                return True
        #no applicable conditions
        return False

    def possible_results(self,state,args={}):
        """Returns all possible results"""
        assert len(args)==len(self.args)
        res = []
        for r in self.results:
            if r.condition.evaluate(state,args,self.context):
                res.append(r)
            else:
                print "Cannot apply action",self.name,",",args,"due to",[e.source_string for e in r.condition.exprs]
        return res

    def next_states(self,state,args={}):
        """Returns a tuple of lists
           [res1,...,resn],[state1,..,staten],[p1,..,pn]
        where resi is the (optional) result name, and pi is the probability
        of ending up in state i for that result."""
        res = self.possible_results(state,args)
        effects = []
        states = []
        ps = []
        for r in res:
            for e,p in zip(r.effects,r.probabilities):
                s = copy.deepcopy(state)
                loc = {'_nextstate_':s,'_state_':state,'_args_':args}
                for i,expr,ass in zip(e.items,e.expressions,e.assignment_expressions):
                    #s[i] = expr.evaluate(state,args)
                    try:
                        exec(ass.parsed_expr,self.context.__dict__,loc)
                    except Exception as e:
                        print "Error evaluating expression",ass.source_string
                        raise
                effects.append(e.name)
                states.append(s)
                ps.append(p)
        #TODO: normalize ps?
        return (effects,states,ps)

class Reward:
    """A reward is a function R(x,a,aresult) where x is the *beginning* state.
    If this reward term is not applicable, it adds a reward of 0."""
    def __init__(self,action=None,conditions=[],value_expr=0,context=None):
        self.action = action
        self.conditions = conditions
        self.value_exprs = [value_expr]
        self.context = context
    def applicable(self,state,action=None,actionargs={},actionresult=None):
        """Returns True if this reward should be assessed on this state"""
        if self.action != action: return False
        for c in self.conditions:
            if c.source_string.isupper():
                #it tests the action result
                if actionresult != c.source_string.strip():
                    return False
            else:
                if not c.evaluate(state,actionargs,self.context):
                    return False
        return True
    def value(self,state,action=None,actionargs={},actionresult=None):
        res = []
        for e in self.value_exprs:
            if isinstance(e,Expression):
                res.append(e.evaluate(state,actionargs,self.context))
            else:
                res.append(e)
        return res

class Problem:
    def __init__(self,context):
        self.actions = {}
        self.rewards = []
        self.context = context
    def applicable_actions(self,state):
        """Returns a list of (action,arg) pairs that could be applicable to the
        given state"""
        res = []
        for a,action in self.actions.iteritems():
            args = action.applicable_args(state)
            for arg in args:
                res.append((a,arg))
        return res
    def parse(self,action_fn,reward_fn):
        actiondb = Database()
        actiondb.read(action_fn)
        assert actiondb.keys[0]=='action'
        assert actiondb.keys[1]=='condition'
        for e in actiondb.entries:
            nameargs = e[0]
            rbasic = re.compile("\w+$")
            rwithargs = re.compile("(\w+)\(([\w ,]+)\)$")
            a = Action(context=self.context)
            #parse name
            if rbasic.match(nameargs):
                a.name = nameargs
            else:
                match = rwithargs.match(nameargs)
                if match == None:
                    raise ValueError("Action name "+nameargs+" is invalid")
                a.name = match.group(1)
                args = match.group(2).split(',')
                for arg in args:
                    arg = arg.strip()
                    a.args[arg] = self.context.domains[arg]
            #parse condition
            if a.name in self.actions:
                a = self.actions[a.name]
            else:
                self.actions[a.name] = a
            aresult = ActionResult()
            aresult.condition = CompoundCondition(e[1])
            for i in range(2,len(e),2):
                if e[i]==None or len(e[i])==0: break
                effect = Effect()
                effect.parse(e[i])
                aresult.effects.append(effect)
                aresult.probabilities.append(Expression(e[i+1].strip()))
            a.results.append(aresult)
        print len(self.actions),"actions read"
        rewarddb = Database()
        rewarddb.read(reward_fn)
        assert(rewarddb.keys[0] == 'action')
        assert(rewarddb.keys[1] == 'condition')
        assert(rewarddb.keys[2] == 'reward')
        rewarddb.cast()
        for e in rewarddb.entries:
            r = Reward(context=self.context)
            r.action = e[0]
            r.conditions = parse_compound_expr(e[1])
            r.value_exprs = []
            for expr in e[2:]:
                if isinstance(expr,str):
                    r.value_exprs.append(Expression(expr))
                else:
                    r.value_exprs.append(expr)
            self.rewards.append(r)
        print len(self.rewards),"rewards read"
        print "Done parsing."
        
    def reward(self,state,action=None,actionargs={},actionresult=None):
        """Assesses the reward of the given action"""
        res = None
        for r in self.rewards:
            if r.applicable(state,action,actionargs,actionresult):
                if res == None:
                    res = r.value(state,action,actionargs,actionresult)
                else:
                    res = vectorops.add(res,r.value(state,action,actionargs,actionresult))
        return res

class Node:
    def __init__(self,state,action,reward=None,parent=None):
        self.state = state
        self.action = action
        self.depth = 0
        self.reward = reward
        self.accumulated_reward = reward
        self.backed_up_score = 0
        self.backed_up_reward = None
        self.best_action = None
        self.parent_result = None
        self.parent = parent
        if parent != None:
            self.depth = parent.depth+1
            self.parent.children.append(self)
            self.accumulated_reward = gen_add(reward,self.parent.accumulated_reward)
        self.children = []
        self.childProbabilities = []
    def destroy(self):
        for c in self.children:
            c.parent = None
            c.destroy()
        self.parent = None
        self.children = []
        self.childProbabilities = []

class Planner:
    def __init__(self,problem,start_state):
        self.problem = problem
        self.root = Node(start_state,None)
    def successors(self,state,action,args):
        """Returns a list of tuples of the form [(r1,s1,p1),...,
        (rk,sk,pk)] where ri is a return value, si is a successor state,
        and pi is a probability."""
        return zip(*self.problem.actions[action].next_states(state,args))
    def visited_state(self,node,s):
        if s == node.state: return True
        if not node.parent: return False
        return self.visited_state(node.parent,s)
    def expand(self,node):
        assert(node.state != None)
        result = []
        #expand actions
        actions = self.problem.applicable_actions(node.state)
        for (a,args) in actions:
            #print "Trying action",a,args
            successors = self.successors(node.state,a,args)
            if len(successors)==0:
                print "Action",a,"no successors?"
                raw_input()
            na = Node(None,(a,args),parent=node)
            for r,s,pexpr in successors:
                #print "Return value",r," gives state",s,"with probability",p
                p = float(pexpr.evaluate(node.state,args,self.problem.context))
                if self.visited_state(node,s):
                    #self loop
                    na.children.append("parent")
                    na.childProbabilities.append(p)
                else:
                    n = Node(s,None,reward=self.problem.reward(node.state,a,args,r),parent=na)
                    n.parent_result = r
                    na.childProbabilities.append(p)
                    result.append(n)
        return result
    def backup(self,node):
        if len(node.children)==0:
            #no children, use reward
            node.best_action = None
            if node.accumulated_reward == None:
                node.backed_up_score = 0
            else:
                node.backed_up_score = self.problem.context.score(*node.accumulated_reward)
            node.backed_up_reward = node.accumulated_reward
            return
        if node.state != None:
            #state node, do a max over actions
            node.best_action = None
            node.backed_up_score = -float('inf')
            for c in node.children:
                self.backup(c)
                if c.backed_up_score > node.backed_up_score:
                    node.best_action = c.action
                    node.backed_up_score = c.backed_up_score
                    node.backed_up_reward = c.backed_up_reward
        else:
            #action node, do an average over children
            node.backed_up_score = 0
            node.backed_up_reward = None
            pselfloop = 0
            for c,p in zip(node.children,node.childProbabilities):
                if c == 'parent':
                    #assume this node goes back to itself
                    pselfloop += p
                else:
                    self.backup(c)
                    node.backed_up_score += p*c.backed_up_score
                    node.backed_up_reward = gen_add(node.backed_up_reward,gen_mul(c.backed_up_reward,p))
            #r = r0 + pselfloop*r
            if pselfloop == 1.0:
                node.backed_up_score = -float('inf')
                node.backed_up_reward = -float('inf')
            else:
                node.backed_up_score /= (1.0-pselfloop)
                node.backed_up_reward = gen_mul(node.backed_up_reward,1.0/(1.0-pselfloop))
        return
    def plan_to_depth(self,depth):
        for c in self.root.children:
            c.destroy()
        self.fringe = deque()
        self.fringe.append(self.root)
        while len(self.fringe)>0:
            n = self.fringe.popleft()
            if n.depth >= depth*2:
                continue
            succ = self.expand(n)
            for s in succ:
                self.fringe.append(s)
        self.backup(self.root)
        print "Resulting score:",self.root.backed_up_score
        print "Resulting reward:",self.root.backed_up_reward
        print "Best initial action:",self.root.best_action
        print "Nominal plan:",self.nominal_plan()
    def nominal_plan(self,node=None):
        if node==None:
            node = self.root
        if node.state != None:
            for c in node.children:
                if node.best_action == c.action:
                    return [node.best_action]+self.nominal_plan(c)
            return []
        else:
            pbest = 0
            cbest = None
            for c,p in zip(node.children,node.childProbabilities):
                if c=='parent':
                    continue
                if p > pbest:
                    cbest = c
                    pbest = p
            if cbest==None:
                return []
            return self.nominal_plan(cbest)
    def pretty_print(self,node=None,indent=0):
        if node==None:
            node = self.root
        if node.state != None:
            if node.parent == None:
                print "  "*indent+"(",node.backed_up_score,node.accumulated_reward,")",node.state
            else:
                print "  "*indent+"(",node.backed_up_score,node.accumulated_reward,")",delta(node.state,node.parent.parent.state)
            for c in node.children:
                self.pretty_print(c,indent+1)
        else:
            assert node.action != None
            if node.action == node.parent.best_action:
                marker = "*"
            else:
                marker = "-"
            print "  "*indent+marker,node.action[0],
            if len(node.action[1]) > 0:
                print "("+','.join(str(k)+'='+str(v) for k,v in node.action[1].iteritems())+")"
            else:
                print
            for c,p in zip(node.children,node.childProbabilities):
                if c == "parent":
                    print "  "*indent+"(",p,") parent self loop"
                else:
                    if isinstance(c,Node) and c.parent_result != None:
                        print "  "*indent+c.parent_result,": (",p,")"
                    else:
                        print "  "*indent+"(",p,")"
                    self.pretty_print(c,indent+1)       
                
if __name__ == '__main__':
    #context.objects = ['object1','object2','object3']
    #context.domains['object'] = context.objects
    #context.bin_contents = {'bin_A':['object2'],'bin_B':['object1','object3']}
    #context.order = ['object1','object2']
    from scoop_context import context
    if len(sys.argv) > 1:
        context.load_from_apc_file(sys.argv[1])
    else:
        context.load_from_apc_file("../../launch_files/apc.json")
    p = Problem(context)
    p.parse("scoop_context/actions.csv","scoop_context/rewards.csv")
    print p.applicable_actions(context.start_state())
    planner = Planner(p,context.start_state())
    planner.plan_to_depth(2)
    raw_input()
    planner.pretty_print()
