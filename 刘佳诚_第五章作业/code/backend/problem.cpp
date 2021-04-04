#include "problem.h"

Problem::Problem(ProblemType problemType){
    problemType_(problemType);
    LogOutVectorSize();
    verticies_marg_.clear();
}