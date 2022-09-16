import pytest

from omegaconf.errors import MissingMandatoryValue
from MotionPlanningEnv.boxObstacle import BoxObstacle
from MotionPlanningSceneHelpers.motionPlanningComponent import ComponentIncompleteError

def test_box_obstacle():
    obst_dict = {'type': 'box',
            'position': [0.1, 0.2, 0.4],
            'geometry': {'length': 0.5,
            'width': 0.5, 'height': 0.5}}
    box_obst= BoxObstacle(name='simpleBox', content_dict=obst_dict)
    assert "simpleBox" == box_obst.name()
    assert [0.1, 0.2, 0.4] == box_obst.position()
    assert 0.5 == box_obst.length()
    assert 0.5 == box_obst.width()
    assert 0.5 == box_obst.height()


def test_error_raise_incomplete_dict():
    obst_dict = {'type': 'box', 'position': [0.1, 0.2], 'geometry': {'length': 0.9}}
    with pytest.raises(MissingMandatoryValue):
        # TODO: er zou geen length geroepen moeten worden....
        box = BoxObstacle(name='simpleBox', content_dict=obst_dict)
        box.height()

def test_length_width_height_incorrect_type():

    for nofloat in [[0.5], [0.5, 0,5], None, {}]:

        obst_dict = {'type': 'box', 'position': [0.1, 0.2, 0.5],
                'geometry': {'length': nofloat,
                'width': 0.5, 'height': 0.5}}
        with pytest.raises(ValueError) as _:
            BoxObstacle(name='simpleBox', content_dict=obst_dict)
        obst_dict = {'type': 'box', 'position': [0.1, 0.2, 0.5],
                'geometry': {'length': 0.5,
                'width': nofloat, 'height': 0.5}}

        with pytest.raises(ValueError) as _:
            BoxObstacle(name='simpleBox', content_dict=obst_dict)
        obst_dict = {'type': 'box', 'position': [0.1, 0.2, 0.5],
                'geometry': {'length': 0.5,
                'width': 0.5, 'height': nofloat}}
        with pytest.raises(ValueError) as _:
            BoxObstacle(name='simpleBox', content_dict=obst_dict)

def test_orientation_incorrect_type():
    obst_dict = {'type': 'box',
            'position': [0.1, 0.2, 0.5],
            'geometry': {'length': 0.5, 'width': 0.5, 'height': 0.5},
            'orientation': ['string', 1, 1, 1]
            }
    with pytest.raises(ValueError) as _:
        BoxObstacle(name='simpleBox', content_dict=obst_dict)

def test_orientation_incorrect_shape():
    obst_dict = {'type': 'box',
            'position': [0.1, 0.2, 0.5],
            'geometry': {'length': 0.5, 'width': 0.5, 'height': 0.5},
            'orientation': [1, 0.4, 0.8, 1, 1, 1]
            }
    with pytest.raises(ValueError) as _:
        BoxObstacle(name='simpleBox', content_dict=obst_dict)

def test_color_incorrect_shape():
    obst_dict = {'type': 'box',
            'position': [0.1, 0.2, 0.5],
            'geometry': {'length': 0.5, 'width': 0.5, 'height': 0.5},
            'color': [1, 0, 1, 1, 1]
            }
    with pytest.raises(ValueError) as _:
        BoxObstacle(name='simpleBox', content_dict=obst_dict)

def test_negative_mass():
    obst_dict = {'type': 'box',
            'position': [0.1, 0.2, 0.5],
            'geometry': {'length': 0.5, 'width': 0.5, 'height': 0.5},
            'mass': -15,
            }
    with pytest.raises(ValueError) as _:
        BoxObstacle(name='simpleBox', content_dict=obst_dict)


def test_color_incorrect_type():
    obst_dict = {'type': 'box',
            'position': [0.1, 0.2, 0.5],
            'geometry': {'length': 0.5, 'width': 0.5, 'height': 0.5},
            'color': ['string', 1, 1, 1]
            }
    with pytest.raises(ValueError) as _:
        BoxObstacle(name='simpleBox', content_dict=obst_dict)
