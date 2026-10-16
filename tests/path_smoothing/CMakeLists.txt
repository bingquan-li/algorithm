add_executable(projected_gradient_test projected_gradient_test.cc)
target_link_libraries(projected_gradient_test
    PRIVATE implementations::path_smoothing
)

foreach(test_case IN ITEMS
    analytic_solution
    finite_difference_gradient
    projection_constraints
    degenerate_problem
    invalid_parameters
    numeric_failure
    verbose_equivalence
    verbose_sampling
    stream_format
    unsupported_backend
)
    add_test(
        NAME path_smoothing.${test_case}
        COMMAND projected_gradient_test ${test_case}
    )
endforeach()
