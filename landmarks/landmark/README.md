# API Guide

Runs on port 5000

## ` /healthcheck ` 
- test if the server is running `"Landmarks Server is Running"`

## ` /getLandmark/<locString> ` 
- Returns the coords if found in dict `{x:5,y:10}`
- If name is similar it returns check for confirmation `{"check", "water cooler"}`
- If the name is not similar is returns error- `{"error": "nothing found"}`

## `/getAllLandmarks`
- Returns all landmarks `{"lift":{"x":5,"y":10,"water cooler":{"x":5,"y":15}}`

## `/setLandmark/<locString>`
- Create a new landmark based on current position: `"success"`

## `/getRelLoc`
- Returns the relative location of the robot
- If you are near something  `"You are near the " + landmark`
- Else `"You are not near anything"`

## `/current`
- Returns the current position
