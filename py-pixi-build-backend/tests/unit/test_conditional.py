from pixi_build_backend.types.conditional import ConditionalString, ListOrItemString


def test_conditional_creation():
    """
    Test creation of a conditional package dependency.
    And validating that it can act as a List.
    
    """
    condition = "os == 'linux'"
    then = ListOrItemString(["package1"])
    else_ = ListOrItemString(["package3"])

    conditional_dep = ConditionalString(condition, then, else_)
    
    assert conditional_dep.condition == condition
    
    print("hello")
    print(conditional_dep.then_value)

    print(conditional_dep.then_value.extend(["package2", "package3"]))
    print(conditional_dep.then_value)
    conditional_dep.then_value[0] = "package001"
    print(conditional_dep.then_value)
    del conditional_dep.then_value[1]

    print(conditional_dep.then_value)
    # print(conditional_dep.then_value.append("package2"))

    print(conditional_dep.then_value)


    then = ListOrItemString(["package001", "package3"])


    assert conditional_dep.then_value == then
    # assert conditional_dep.else_value == else_