#ifndef PIPELINE_HPP
#define PIPELINE_HPP

namespace ConstraintSolving {

class Detector {
public:
    virtual bool isGcsSolvable() = 0;
};

class Resolver {
    virtual void resolveGcs() = 0;
};

class Decomposer {
    virtual void decompeGcs() = 0;
};

class Solver {
    virtual void solveGcs() = 0;
};

class PipeLine {
};

};

#endif // PIPELINE_HPP
