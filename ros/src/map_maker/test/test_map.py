from array import array

from map_maker import map_maker

from nav_msgs.msg import OccupancyGrid, MapMetaData


def test_map_set():
    tests = [
        {
            "input": {"width": 8, "height": 5, "change": [2, 2, 1]},
            "output": array(
                "b",
                [
                    0,
                    0,
                    0,
                    0,
                    0,
                    0,
                    0,
                    0,
                    0,
                    0,
                    0,
                    0,
                    0,
                    0,
                    0,
                    0,
                    0,
                    0,
                    1,
                    0,
                    0,
                    0,
                    0,
                    0,
                    0,
                    0,
                    0,
                    0,
                    0,
                    0,
                    0,
                    0,
                    0,
                    0,
                    0,
                    0,
                    0,
                    0,
                    0,
                    0,
                ],
            ),
        },
        {
            "input": {"width": 3, "height": 7, "change": [2, 4, 1]},
            "output": array(
                "b", [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0]
            ),
        },
        # the invalid point test
        {
            "input": {"width": 8, "height": 3, "change": [20, 4, 1]},
            "output": array(
                "b",
                [
                    0,
                    0,
                    0,
                    0,
                    0,
                    0,
                    0,
                    0,
                    0,
                    0,
                    0,
                    0,
                    0,
                    0,
                    0,
                    0,
                    0,
                    0,
                    0,
                    0,
                    0,
                    0,
                    0,
                    0,
                ],
            ),
        },
    ]

    for test in tests:
        input_test = test["input"]
        expected = test["output"]

        og_map = OccupancyGrid()
        meta = MapMetaData()

        meta.height = input_test["height"]
        meta.width = input_test["width"]

        og_map.data = [0] * meta.height * meta.width
        og_map.info = meta
        map_maker.update_map(og_map, input_test["change"][:2], input_test["change"][2])
        # print(og_map.data)
        # compare_array(expected, og_map.data)
        assert expected == og_map.data


def test_robo_point():
    tests = [
        {
            "input": [
                [2, 40],
                [50, 40],
                [50, 80],
                [2, 80],
            ],
            "output": (26, 60),
        },
        # rotated
        {"input": [(37, 31), (64, 44), (57, 68), (29, 55)], "output": (47, 50)},
    ]

    for test in tests:
        test_input = test["input"]
        expected = test["output"]
        point = map_maker.calculate_pos(test_input)
        # print(og_map.data)
        # compare_array(expected, og_map.data)
        assert expected == point
