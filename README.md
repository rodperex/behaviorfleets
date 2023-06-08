# behaviorfleets

TO DO:
    - Enhance protocol so when a node is excluded, it is communicated so it does not try anymore (stops potential channel block). Makes sense?

SHARED BB:
    - Centralized node that manages the BB
    - Every remote robot has its own handler to keep the local BB sychronized with the global

LIMITATION TO SOLVE:
    - Now, when sharing blackboard, all keys should be added as strings since the global blackboard is propagated as map of strings. So if the local blackboard tries to updato one of the keys, let's say with a in integer number, we would have a type error. Thus it is necessary to parse it to string.