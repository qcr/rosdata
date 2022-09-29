#!/usr/bin/env python

################
### MODULES ####
################

import pathlib


################
### CLASSES ####
################


########################
### PUBLIC FUNCTIONS ###
########################

def yes_or_no(question: str):
    """Yes or no question from gist
        https://gist.github.com/garrettdreyfus/8153571

    Args:
        question (str): Question to ask

    Returns:
        bool: Returns true or false to question
    """
    reply = str(input(question+' (y/n): ')).lower().strip() #Just look at first char
    if len(reply):
        if reply[0] == 'y':
            return True
        if reply[0] == 'n':
            return False
        else:
            return yes_or_no("Uhhhh... please enter ") #Invalid input
    else:   
        return yes_or_no("Uhhhh... please enter ") #Zero length


def rmdir(dir: pathlib.Path):
    """Removes a directory and any sub-directories

    Args:
        dir (Path): directory to be removed
    """
    for item in dir.iterdir():
        if item.is_dir():
            rmdir(item)
        else:
            item.unlink()
    dir.rmdir()