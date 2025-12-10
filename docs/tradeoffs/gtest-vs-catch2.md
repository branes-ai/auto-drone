# Google Test vs Catch2

Walk-through of the **matrix multiplication test case** in both frameworks so you can see the practical differences in syntax and workflow.

---

## GoogleTest Example

```cpp
#include <gtest/gtest.h>
#include <vector>

// Simple matrix multiplication function
std::vector<std::vector<int>> matmul(const std::vector<std::vector<int>>& A,
                                     const std::vector<std::vector<int>>& B) {
    int n = A.size(), m = B[0].size(), k = B.size();
    std::vector<std::vector<int>> C(n, std::vector<int>(m, 0));
    for (int i = 0; i < n; ++i)
        for (int j = 0; j < m; ++j)
            for (int p = 0; p < k; ++p)
                C[i][j] += A[i][p] * B[p][j];
    return C;
}

TEST(MatrixMultiplicationTest, SmallCase) {
    std::vector<std::vector<int>> A = {{1, 2}, {3, 4}};
    std::vector<std::vector<int>> B = {{2, 0}, {1, 2}};
    auto C = matmul(A, B);

    EXPECT_EQ(C[0][0], 1*2 + 2*1);  // 4
    EXPECT_EQ(C[0][1], 1*0 + 2*2);  // 4
    EXPECT_EQ(C[1][0], 3*2 + 4*1);  // 10
    EXPECT_EQ(C[1][1], 3*0 + 4*2);  // 8
}
```

- **Style:** Macro-driven (`TEST`, `EXPECT_EQ`)  
- **Strength:** Clear separation of test cases, fixtures possible for larger setups  
- **Integration:** Works well with CI/CD pipelines and mocking frameworks  

---

## Catch2 Example

```cpp
#define CATCH_CONFIG_MAIN
#include <catch2/catch.hpp>
#include <vector>

std::vector<std::vector<int>> matmul(const std::vector<std::vector<int>>& A,
                                     const std::vector<std::vector<int>>& B) {
    int n = A.size(), m = B[0].size(), k = B.size();
    std::vector<std::vector<int>> C(n, std::vector<int>(m, 0));
    for (int i = 0; i < n; ++i)
        for (int j = 0; j < m; ++j)
            for (int p = 0; p < k; ++p)
                C[i][j] += A[i][p] * B[p][j];
    return C;
}

TEST_CASE("Matrix multiplication works", "[matmul]") {
    std::vector<std::vector<int>> A = {{1, 2}, {3, 4}};
    std::vector<std::vector<int>> B = {{2, 0}, {1, 2}};
    auto C = matmul(A, B);

    REQUIRE(C[0][0] == 4);
    REQUIRE(C[0][1] == 4);
    REQUIRE(C[1][0] == 10);
    REQUIRE(C[1][1] == 8);
}
```

- **Style:** Expression-based (`REQUIRE`) feels like natural C++  
- **Strength:** Minimal boilerplate, easy to read and write  
- **Integration:** Lightweight, header-only (v2), good for prototyping  

---

## Key Takeaway
- **GoogleTest**: More structured, verbose, but powerful for large projects with fixtures and mocks.  
- **Catch2**: Cleaner syntax, faster to write, great for readability and smaller projects.  

Great—let’s extend the matrix multiplication example into **parameterized test cases** so you can see how each framework scales when validating multiple matrix sizes or configurations. This is especially useful when benchmarking tiling strategies or verifying correctness across non-square matrices.

---

## GoogleTest Parameterized Example

```cpp
#include <gtest/gtest.h>
#include <vector>

std::vector<std::vector<int>> matmul(const std::vector<std::vector<int>>& A,
                                     const std::vector<std::vector<int>>& B) {
    int n = A.size(), m = B[0].size(), k = B.size();
    std::vector<std::vector<int>> C(n, std::vector<int>(m, 0));
    for (int i = 0; i < n; ++i)
        for (int j = 0; j < m; ++j)
            for (int p = 0; p < k; ++p)
                C[i][j] += A[i][p] * B[p][j];
    return C;
}

// Define a struct for test parameters
struct MatMulParam {
    std::vector<std::vector<int>> A;
    std::vector<std::vector<int>> B;
    std::vector<std::vector<int>> expected;
};

class MatMulTest : public ::testing::TestWithParam<MatMulParam> {};

TEST_P(MatMulTest, WorksForVariousSizes) {
    auto param = GetParam();
    auto C = matmul(param.A, param.B);
    EXPECT_EQ(C, param.expected);
}

// Instantiate with multiple cases
INSTANTIATE_TEST_SUITE_P(
    MatrixCases,
    MatMulTest,
    ::testing::Values(
        MatMulParam{{{1,2},{3,4}}, {{2,0},{1,2}}, {{ {4,4}, {10,8} }}},
        MatMulParam{{{1,0,2}}, {{0,1},{1,0},{2,1}}, {{ {5,3} }}}
    )
);
```

- **Strength:** Scales cleanly with `INSTANTIATE_TEST_SUITE_P` for multiple datasets.  
- **Best for:** Large test matrices, CI pipelines, and systematic validation across many configurations.  

---

## Catch2 Parameterized Example

```cpp
#define CATCH_CONFIG_MAIN
#include <catch2/catch.hpp>
#include <vector>

std::vector<std::vector<int>> matmul(const std::vector<std::vector<int>>& A,
                                     const std::vector<std::vector<int>>& B) {
    int n = A.size(), m = B[0].size(), k = B.size();
    std::vector<std::vector<int>> C(n, std::vector<int>(m, 0));
    for (int i = 0; i < n; ++i)
        for (int j = 0; j < m; ++j)
            for (int p = 0; p < k; ++p)
                C[i][j] += A[i][p] * B[p][j];
    return C;
}

TEST_CASE("Matrix multiplication parameterized", "[matmul]") {
    SECTION("2x2 case") {
        std::vector<std::vector<int>> A = {{1,2},{3,4}};
        std::vector<std::vector<int>> B = {{2,0},{1,2}};
        auto C = matmul(A, B);
        REQUIRE(C == std::vector<std::vector<int>>{{4,4},{10,8}});
    }

    SECTION("1x3 * 3x2 case") {
        std::vector<std::vector<int>> A = {{1,0,2}};
        std::vector<std::vector<int>> B = {{0,1},{1,0},{2,1}};
        auto C = matmul(A, B);
        REQUIRE(C == std::vector<std::vector<int>>{{5,3}});
    }
}
```

- **Strength:** Uses `SECTION` to group multiple scenarios under one test case.  
- **Best for:** Readable, lightweight parameterization without boilerplate.  

---

## Key Difference
- **GoogleTest:** Explicit parameterization with `TestWithParam` and `INSTANTIATE_TEST_SUITE_P`. More structured, ideal for scaling to dozens or hundreds of datasets.  
- **Catch2:** Relies on `SECTION` blocks. Simpler, but less formalized—great for a handful of scenarios, less ideal for massive test suites.  

---

Validating **non-square and massive matrices on modest hardware fabrics**, GoogleTest’s parameterization might provide more leverage for **systematic benchmarking across tiling strategies**. Catch2’s `SECTION` style is elegant for **quick exploratory validation** when iterating IR prototypes.  


