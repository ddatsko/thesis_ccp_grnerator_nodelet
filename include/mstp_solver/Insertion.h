//
// Created by mrs on 28.03.22.
//

#ifndef THESIS_TRAJECTORY_GENERATOR_INSERTION_H
#define THESIS_TRAJECTORY_GENERATOR_INSERTION_H

struct Insertion {
    double solution_cost;
    size_t target_set_index;
    size_t target_index;
    size_t uav_index;
    size_t insertion_index;
};

struct InsertionComp {
    bool operator()(const Insertion &i1, const Insertion &i2) const {
        return i1.solution_cost < i2.solution_cost;
    }
};


#endif //THESIS_TRAJECTORY_GENERATOR_INSERTION_H
