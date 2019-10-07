
Feature: Movement
  I want to test expected movement against actual movement given a set of simple commands

  Scenario: Move forward 1m
    Given I have a robot
    And the robot started at 0,0,0
    And I tell the robot to move forward 1m
    Then the robot should now be at 0,0,0
