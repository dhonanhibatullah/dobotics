from .submodule.access_test_parent import *

def main(args=None):
    print(MESSAGE_PARENT)
    print(MESSAGE_CHILD)
    print(MESSAGE_PARENT_PEER)
    print(MESSAGE_CHILD_PEER)

if __name__ == '__main__':
    main()