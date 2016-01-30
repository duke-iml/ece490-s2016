def print_instance_var(o):
    print [i for i in dir(o) if "__" not in i]
