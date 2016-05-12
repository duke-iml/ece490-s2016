JSON Test File Generator:
------------------------
Author:
    Dennis Lynch

License:
    This software can be used freely under the MIT License

Uses:
    This generator is designed to create test JSON files using the list of APC items provided for the 2016 event, and modeled off of the sample apc2015 json file available in the Duke ECE490 Spring 2016 APC repository.  This program provides two modes of file completion:
    - Random Mode
        - The terminal will prompt the user to define how many items they wish to add to the json file, and then randomly assign items from the list of applicable APC items to the shelf before writing out the file
    - Interactive Mode
        - The terminal will prompt the user with option to view available items, view shelf contents, remove items from the shelf, add items to the shelf, and make a file based on the current contents of the shelf
        - When adding or removing items from the shelf, standard linux autocomplete is enabled to make adding or removing items easier

How to run:
    Run the command
        python json_test_maker.py

    The terminal will prompt the user from there with the proper usage of the program

Issues:
    Please report any issues via slack
