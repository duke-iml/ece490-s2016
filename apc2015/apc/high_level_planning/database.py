import csv
import copy
import re
import math

class Database:
    """
    A simple associative database.

    Attributes:
    - keys: names of entries in the database
    - entries: rows in the database.  These are stored as lists, which may
      contain None for missing entries.

    To get a Numpy array, simply call np.array(entries).

    Can write query conditions given by a string of the form
    'expr({arg1},...,{argn})' where the bracketed arguments are
    database keys.  These can be given to delete_if(cond), get_if(cond)
    
    Transform operations are given by expressions of the form
    'name = expr({arg1},...,{argn})' where the bracketed arguments are
    database keys.  These can be given to process(expr)

    The string '_e_' cannot be used in the expressions.
    """
    
    def __init__(self,*args):
        """Can be initialized with either a Database or a
        keys,entries pair."""
        self.keys = []
        self.entries = []
        if len(args) == 1:
            self.keys = args[0].keys[:]
            self.entries = copy.deepcopy(args[0].entries)
        elif len(args) == 2:
            self.keys = args[0][:]
            self.entries = copy.deepcopy(args[1])

    def read(self,fn):
        """Reads from a file.  Currently only supports CSV format.
        Note: stores entries as strings.  To cast to the appropriate numeric
        type, call cast() after this returns"""
        self.readCSV(fn)

    def write(self,fn):
        """Writes to a file.  Currently only supports CSV format"""
        self.writeCSV(fn)
        
    def readCSV(self,fn):
        """Reads from a CSV file.  Assumes that the first line contains the
        keys.  Note: stores entries as strings.  To cast to the appropriate
        numeric type, call cast() after this returns."""
        self.keys = []
        self.entries = []
        f = open(fn,'r')
        reader = csv.reader(f)
        for lineno,line in enumerate(reader):
            if lineno == 0:
                self.keys = line[:]
            else:
                self.entries.append(line)
        f.close()
        
    def writeCSV(self,fn):
        """Writes to a CSV file.  Assumes that the first line contains the
        keys"""
        f = open(fn,'w')
        writer = csv.writer(f)
        writer.writerow(self.keys)
        for pt in self.entries:
            writer.writerow(pt)
        f.close()

    def copy(self):
        """Returns a (deep) copy of this."""
        return Database(self)
        
    def get(self,index,key):
        """Returns the entry in row 'index' and key 'key'."""
        keyindex = self.keys.index(key)
        return self.entries[index][keyindex]
    def set(self,index,key,v):
        """Sets the entry in row 'index' and key 'key' to the value in 'v'."""
        keyindex = self.keys.index(key)
        self.entries[index][keyindex] = v
    def column(self,key):
        """Gets a list of all entries with key 'key'."""
        keyindex = self.keys.index(key)
        return [e[keyindex] for e in self.entries]
    def set_column(self,key,values):
        """Sets all entries with key 'key' to the corresponding value in
        the list values."""
        keyindex = self.keys.index(key)
        assert len(values)==len(self.entries)
        for e,v in zip(self.entries,values):
            e[keyindex] = v
    def cast(self,key=None):
        """Casts entries of the given key to the most appropriate type"""
        if key == None:
            types = self.type(None)
            for e in self.entries:
                for (i,t) in enumerate(types):
                    e[i] = t(e[i])
        else:
            index = self.keys.indx(key)
            type = self.type(key)
            for e in self.entries:
                e[index] = type(e[index])
    def type(self,key=None):
        """Finds the most appropriate type for the given key"""
        if key == None:
            #return a list of types
            return [self.type(k) for k in self.keys]
        else:
            index = self.keys.index(key)
            vals = [e[index] for e in self.entries]
            ints = 0
            floats = 0
            for v in vals:
                if isinstance(v,int):
                    ints += 1
                elif isinstance(v,float):
                    floats += 1
                else:
                    try:
                        temp = int(v)
                        ints += 1
                    except:
                        try:
                            temp = float(v)
                            floats += 1
                        except:
                            return str
            if ints == len(vals):
                return int
            elif ints + floats == len(vals):
                return float
    def subst(self,statement,prefix):
        """Used internally."""
        items = re.split("(\{.+?\})",statement)
        res = ""
        for r in items:
            if len(r)==0: continue
            if r[0]=='{' and r[-1]=='}':
                k = r[1:-1]
                if k not in self.keys:
                    raise KeyError('Key "%s" not in database'%(str(k),))
                res = res+prefix+"["+str(self.keys.index(k))+"]"
            else:
                res = res+r
        return res
    def delete_key(self,*argv):
        """Deletes all of the given keys"""
        for v in argv:
            if v not in self.keys:
                raise KeyError("Key "+str(v)+" not in database")
            ind = self.keys.index(v)
            self.keys.pop(ind)
            for e in self.entries:
                e.pop(ind)
    def delete_if(self,cond,data={}):
        cond = self.subst(cond,'_e_')
        newList = []
        for i,e in enumerate(self.entries):
            locals = {'_e_':e}
            locals.update(data)
            if not eval(cond,globals(),locals):
                newList.append(e)
        self.entries = newList
    def first_if(self,cond,data={}):
        """Returns the index of the first entry of this database such that cond(entry) is true
        or None if none could be found."""
        cond = self.subst(cond,'_e_')
        for i,e in enumerate(self.entries):
            locals = {'_e_':e}
            locals.update(data)
            if eval(cond,globals(),locals):
                return i
        return None
    def last_if(self,cond,data={}):
        """Returns the last entry of this database such that cond(entry) is true
        or None if none could be found."""
        cond = self.subst(cond,'_e_')
        for i,e in enumerate(reversed(self.entries)):
            locals = {'_e_':e}
            locals.update(data)
            if eval(cond,globals(),locals):
                return len(self.entries)-i-1
        return None
    def get_if(self,cond,data={}):
        """Returns a subset of this database such that cond(entry) is true."""
        cond = self.subst(cond,'_e_')
        db = Database()
        db.keys = self.keys[:]
        for i,e in enumerate(self.entries):
            locals = {'_e_':e}
            locals.update(data)
            if eval(cond,globals(),locals):
                db.entries.append(e)
        return db
    def get_keys(self,keys):
        """Returns a new Database object with only the selected keys"""
        db = Database()
        indices = [self.keys.index(k) for k in keys]
        db.keys = keys[:]
        for e in self.entries:
            db.entries.append([e[ind] for ind in indices])
        return db
    def shuffle_keys(self,keyorder):
        """Rearranges the order of the keys given the new key order.
        This function can also be used to select a subset of the keys
        by setting keyorder to be a subset of the keys."""
        indices = [self.keys.index(k) for k in keyorder]
        self.keys = keyorder[:]
        for i,e in enumerate(self.entries):
            self.entries[i] = [e[ind] for ind in indices]
    def join(self,db):
        """A simple join of the two databases formed just by concatenating all
        entries of db onto the corresponding entries of this one."""
        assert len(self.entries) == len(db.entries)
        self.keys = self.keys + db.keys
        for (e1,e2) in zip(self.entries,db.entries):
            e1.extend(e2)
    def process(self,assignments,data={}):
        """Joins a transform(assignments) into self by adding the transformed
        items onto each entry in the self database.  If the transformed
        item has the same name as a key in self, then that entry's attribute
        according to that key is replaced."""
        db = None
        if hasattr(assignments,'__iter__'):
            db=self.transform(assignments,data)
        else:
            db=self.transform([assignments],data)
        indices = []
        for k in db.keys:
            if k in self.keys:
                indices.append(self.keys.index(k))
            else:
                self.keys.append(k)
                indices.append(len(self.keys)-1)
        for e1,e2 in zip(self.entries,db.entries):
            while len(e1) < len(self.keys):
                e1.append(None)
            for (i,v) in zip(indices,e2):
                e1[i] = v
            assert len(e1)==len(self.keys)
    def split(self,key):
        """Splits the db into separate databases given the values of the key"""
        if self.type(key) == float:
            raise ValueError("Can't split on floating point attributes")
        index = self.keys.index(key)
        vals = dict()
        for e in self.entries:
            vals.setdefault(e[index],[]).append(e)
        res = [None]*len(vals)
        for (i,v) in enumerate(vals.itervalues()):
            res[i] = Database()
            res[i].keys = self.keys[:]
            res[i].entries = copy.deepcopy(v)
        return res
    def transform(self,assignments,data={}):
        """Assignments can be a command of the form 'name = expr(items)'
        where name is the name of the new entry, and expr(keys) is some
        python expression in which keys can be referenced using the string
        {key}.  E.g. 'foo = -{bar}*3'.  Multiple assignments can also be
        given as a list"""
        db = Database()
        commands = []
        for assignment in assignments:
            rhs,lhs = assignment.split('=',1)
            rhs = rhs.strip()
            db.keys.append(rhs)
            lhs = self.subst(lhs,'_e_')
            commands.append(lhs)
        types = self.type()
        for e in self.entries:
            evals = [t(v) for t,v in zip(types,e)]
            locals = {'_e_':evals}
            locals.update(data)
            newentry = []
            for cmd in commands:
                try:
                    newentry.append(eval(cmd,globals(),locals))
                except Exception:
                    print "Unable to evaluate",cmd,"in env",locals
                    newentry.append(None)
            db.entries.append(newentry)
        return db
    def append(self,db):
        """Appends the entries of a database onto this one.  Does a little
        more error checking than would appending the entries."""
        if len(self.keys) == 0:
            self.keys = db.keys[:]
        assert (self.keys == db.keys)
        self.entries = self.entries + [e[:] for e in db.entries]
    def range(self,entry=None):
        """Returns the (min,max) of the given entry.  If entry is not given,
        then all of the ranges will be returned."""
        if entry == None:
            #return a list of ranges
            return [self.range(k) for k in self.keys]
        else:
            t = self.type(entry)
            if t == str:
                return None
            index = self.keys.index(entry)
            return min(e[index] for e in self.entries),max(e[index] for e in self.entries)
    def mean(self,entry=None):
        if entry == None:
            #return a list of means
            return [self.mean(k) for k in self.keys]
        else:
            t = self.type(entry)
            if t == str:
                return None
            index = self.keys.index(entry)
            return sum([float(e[index]) for e in self.entries])/len(self.entries)
    def stdev(self,entry=None):
        if entry == None:
            #return a list of stdevs
            return [self.stdev(k) for k in self.keys]
        else:
            t = self.type(entry)
            if t == str:
                return None
            index = self.keys.index(entry)
            mean = self.mean(entry)
            ss = sum([pow(float(e[index])-mean,2) for e in self.entries])
            return math.sqrt(ss/len(self.entries))
    def whiten(self,entry=None):
        """Normalizes the given entry to be zero mean, unit variance by
        subtracing the mean and dividing by the standard deviation. If entry
        is not given, then all entries are whitened."""
        if entry == None:
            for k in self.keys:
                self.whiten(k)
        else:
            t = self.type(entry)
            if t == str:
                return
            index = self.keys.index(entry)
            mean = self.mean(entry)
            stdev = self.stdev(entry)
            if stdev == 0.0: return
            for e in self.entries:
                e[index] = (float(e[index])-mean)/stdev
