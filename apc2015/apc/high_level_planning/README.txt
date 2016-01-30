HIGH LEVEL PLANNER DOCUMENTATION
5/14/2015
Kris Hauser

This documents the high level planning code used for APC.

CURRENT DEFINITION

State variables:
- object: the name of the target object we're currently trying to pick
- bin: the name of the target bin we're currently trying to pick from
- at_bin: true if the hand is moved to the bin
- recognized_object: None if the recognizer hasn't been run.  Otherwise, the
  name of the recognized object, or 'other' if some other object was
  mistakenly identified.
- good_pose_estimate: None if the pose estimator was not run.  True/False
  depending on whether the pose estimate was high-quality or not.
- good_segmentation: None if the segmenter was not run.  True/False
  depending on whether the segmentation is high-quality or not.
- grasped_object: None if there is no object in the hand, otherwise the
  name of the grasped object.  Note this may not be the target object.
- grasp_failures: an integer indicating how many times GRASP was called on the
  current object.
- pc_grasp_failures: an integer indicating how many times PC_GRASP was called
  on the current object.
- pick_done: True if the object was released from the hand.  Just used to
  trigger a reset on the problem.

Actions:
- START_PICK(bin,object): choose to pick up the given object from the given 
  bin.
- MOVE_TO: goes to the chosen bin.
- RECOGNIZE: tests whether the object can be detected in the current bin.
  Has some chance of failure, especially on multi object bins.
- SEGMENT: segments the object.  Has a chance of failure that rises with more
  objects in the bin.  
- ESTIMATE_POSE: estimates the pose of the object.  Has a chance of failure
  that increases when there are more objects in the bin.
- GRASP: runs an ICP-based grasp on the object.  If successful, the object
  is grasped.
- PC_GRASP: runs the point cloud-based grasp on the object.  If successful,
  the object is grasped.
- SCOOP: runs the scooper.  If successful, the object is grasped.
- PLACE: puts a grasped object into the order bin.  May also drop the object.
- QUIT_PICK: premature quit of trying to pick up the current target object
- STOP_PICK: normal termination with a successful pick (or drop).

TODO: add more accurate outcome probabilities and object-specific
probabilities from testing.



MODIFYING THE PROBLEM

The state of the system is represented by a state variable with named elements.  Actions have a list of preconditions which are tested with respect to the state at which they are applied.  Once an action is applied, the state variables are modified.

This makes it really easy to add new actions, state variables, and rewards.

ACTIONS

In actions.csv, you can add a new action by adding an entry "FUNC" to the first column, and it optionally can have arguments "FUNC(ARG1,ARG2,…)".  The domains of the arguments must be given in the domains dictionary in the context.py module.

The second column has a semicolon-separated Python expression "EXPR" which must be satisfied in the current state.  References to variables in the current state are given by the format "{var}". References to arguments are given by the format "$arg". 

Columns (3,4), (5,6), etc define possible outcomes of the action and their probabilities of being arrived in.  They are semicolon-separated lists of the form "var = EXPR".  The state variable "var" changes to the result of evaluating the python expression EXPR.  The list can also be prepended with a single all-caps identifier that names the outcome of the action (like SUCCESS, FAILURE, etc).  At the moment probabilities are only single numbers but I can change them to expressions if you feel like it is necessary.  

Note: tweaking the actions is rather sensitive and error-prone, because you have to input a whole host of conditions that restricts the applicability of your action to the context in which it is supposed to be applied.  It is recommended to back up the planning context before making major changes.


STATE VARIABLES

The initial state is given when you create the planner.  Afterwards, state variables are defined implicitly as the result of applying actions to states. 

By default, the initial state is given by the dictionary start_state in context.py


REWARDS

In rewards.csv, you can define rewards and costs that are assessed whenever an action is performed under a given condition.  The conditions are again python expression lists.  You can also refer to certain outcomes (e.g., penalize FAILURE and reward SUCCESS outcomes).

To score a trajectory, the planner sums up the rewards and costs, and the ultimate score is the ratio of summed rewards to summed costs (r1+...+rn)/(c1+...+cn)



USAGE

Run "python hlp.py" to test the planner.  It will generate a plan for the current planning context.  In the system-level code you can import Planner and Problem objects, and run them by hand.  

Note that the plan is actually a policy tree; ostensibly you would take the first step, observe the result of the plan, and follow the appropriate branch of the policy tree to get the next step.  If this is too difficult, you may simply take the *nominal* plan, which is the single path of maximum probability. 

Recommended depth of the search tree is approximately 7-8 actions.  Any further and the planner takes a very long time to complete.  The planner should be called once at the beginning to choose an object to pick and a recommended perception strategy. The planner need not be called again unless some surprising observations are found (e.g., perception or planning failure) or the plan completes.
