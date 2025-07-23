//
// Created by Hugo Trippaers on 25/10/2024.
//

#include "unity_fixture.h"

TEST_GROUP_RUNNER(AdditionalFunctions)
{
    RUN_TEST_CASE(AdditionalFunctions, SkewMatrix)

    RUN_TEST_CASE(AdditionalFunctions, TransformationMatrix)

    RUN_TEST_CASE(AdditionalFunctions, MatrixScalarMult)

    RUN_TEST_CASE(AdditionalFunctions, VectorCrossProduct)
    RUN_TEST_CASE(AdditionalFunctions, VectorCrossProduct2)

    RUN_TEST_CASE(AdditionalFunctions, VectorNorm)

    RUN_TEST_CASE(AdditionalFunctions, VectorNormalize)
    RUN_TEST_CASE(AdditionalFunctions, VectorNormalize2)
}
