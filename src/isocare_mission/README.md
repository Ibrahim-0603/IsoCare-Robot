# isocare_mission

`mission_manager` runs one visit to one bed. `demo_stubs` stands in for the nodes that
do not exist yet. Draft interfaces live in `isocare_interfaces` (`action/*.action`,
`srv/StartVisit.srv`) and are to be reviewed with Youssef Hesham and Youssef Eslam.

Visit: GO_TO_BED, IDENTIFY, MEASURE, DECIDE, WAIT_APPROVAL, DISPENSE, RETURN.
Any failure: ALERT (drawers locked, sensors off, navigation cancelled, nurse told),
then GOING_HOME. See `mission_core.py` for the rules; every number is a parameter.
