//
// Created by kandithws on 7/6/2560.
//

#ifndef CGR_GRIDMAP_SLAM_ROS_PARAMETERS_MACRO_H
#define CGR_GRIDMAP_SLAM_ROS_PARAMETERS_MACRO_H

// Create GET SET for params
#define CGR_PARAM(NAME, UNDERSCORED, TYPE, DEFAULT_VAL)\
private: TYPE UNDERSCORED##_ = DEFAULT_VAL; \
public: inline TYPE get##NAME(){return UNDERSCORED##_; }\
        inline void set##NAME(TYPE value){UNDERSCORED##_ = value;}

// Create GET SET interface for params inside member_variables_
#define CGR_MEMBER_VAR_PARAM(MEMBER_VAR, NAME, TYPE)\
public: inline TYPE get##NAME(){return MEMBER_VAR.get##NAME();}\
        inline void set##NAME(TYPE value){MEMBER_VAR.set##NAME(value);}

#endif //CGR_GRIDMAP_SLAM_ROS_PARAMETERS_MACRO_H
