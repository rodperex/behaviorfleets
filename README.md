# behaviorfleets

TO DO:
    - Number of updates from the global bb correspond to the summatory of all successful updates from other nodes
    - Make the stressers work until all requests have been attended?
    - When there are many nodes working, only a couple of requests are attended during the operation time
    - Instead of providing operation time, we can provide number os successful updates as a condition to stop


Measuring waiting time in the server does not isolate the measurement from the client Hz since the bb is locked till the client releases it.