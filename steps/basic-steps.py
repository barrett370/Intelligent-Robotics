from behave import given, when, then

from src.foo.main import Robot, Coordinates


@given('I have a robot')
def init_robot(context):
    context.robot = Robot


@given('the robot started at {x:d},{y:d},{z:d}')
def starting(context, x: int, y: int, z: int):
    context.robot.set_coords(context.robot, x, y, z)


@given('I tell the robot to move forward {number:d}m')
def dist(context, number: int):
    context.robot.move_forward(context.robot, number)
    context.result = context.robot.coords


@then('the robot should now be at {x2:d},{y2:d},{z2:d}')
def final_pos(context, x2: int, y2: int, z2: int):
    final_coords = Coordinates(x2, y2, z2)
    print(context.result, final_coords)
    assert context.result == final_coords
