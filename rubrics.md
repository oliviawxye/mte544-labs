## Part 1 and 2: No marks

## Part 3: 10 marks
**In-Lab:**
- ```10``` marks for successful map acquisition and saving as `room` map.

## Part 4: 40 marks
**Codes: 20 marks**
- ```5``` marks for finishing the motion model in `particle.py -> motion_model`.
- ```5``` marks for finishing the random particle generation for initialization in `particleFilter.py -> initializeParticleFilter`.
- ```5``` marks for finishing the random particle sampling in `particleFilter.py -> resample`.
- ```5``` marks for finishing the particle filter adding noise to the sampled particles in `particleFilter.py -> resample`.

**In-Lab: 20 marks**
- ```5``` marks for explaining the likelihood computation in `mapUtilities.py -> make_likelihood_field` to a TA.
- ```5``` marks for explaining how to compute the weight for each particle in `particle.py -> calculateParticleWeight` to a TA.
- ```10``` marks for demonstrating the particle filter pose estimation in Rviz while moving the robot via keyboard.

## Part 5: 20 marks
**Codes: 10 marks**
- ```10``` marks for correctly logging the pose in `localization.py -> odom_and_pf_pose_callback`.

**In-Lab: 10 marks**
- ```10``` marks for integrating the particle filter with the navigation stack and demonstrating particle filter-based localization in Rviz while the robot moves to a goal using the point controller.

## Conclusions: 25 marks
**From Report on LEARN:**
- ```1``` mark for names (Family Name, First Name); student IDs; station number; and robot number of all group members.
- ```10``` marks for plots with title, label names for axes, legends, different shapes/colors for each data, and grids.
- ```7``` marks for discussion comparing the particle filter and raw sensor method in the light of the plotted data.
- ```7``` marks for discussion on the performance comparison by changing the laser scan deviation.

## Pre-lab Deliverable: 5 marks
- ```5``` marks for submitting a first version of the completed code at least 24 hours before the group's lab section.
- Failure to submit results in a ```5``` mark penalty on the final lab report.
- The code does not have to be fully correct but should be nearly complete and meaningful with appropriate comments.

## Submission Requirements
- **Report**: Submit one single PDF file containing all required sections and plots.
- **Code**: Submit a single zip file containing all source files, including CSV files obtained from data logging.
- Only one submission per group is needed.

Good luck!

