# MBLUE-PEA

# Design Stage 1: Stiffness Profile Generation
a software tool that facilitates the design of lightweight torsion springs

Before choosing a spring or designing a cam surface, we must determine what torque-angle relationship to render. This consists of the following steps: 

Task Selection: Occurs at the top of OptimizationWrapper.m

  Define how many tasks to optimize for, and what those tasks are

  Dataset Selection: we are using the Georgia Tech 2023 Dataset, for which we have taken all-subject averages

  Parameter Selection (assistance fractions, task weights, subject mass)

  Define the joint that will be assisted and what actuator will be used. 
  If new datasets or actuators are to be used, update the getGaitData.m or getActuatorData.m to reflect the change. 
