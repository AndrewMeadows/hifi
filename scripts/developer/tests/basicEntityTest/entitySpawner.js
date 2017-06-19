  var orientation = Camera.getOrientation();
  orientation = Quat.safeEulerAngles(orientation);
  orientation.x = 0;
  orientation = Quat.fromVec3Degrees(orientation);
  var center = Vec3.sum(MyAvatar.position, Vec3.multiply(3, Quat.getForward(orientation)));

  //TODO_CUSACK: Return this to its pre-21389 state after
  //             testing.  Should make a custom one for QA to use.
  // Math.random ensures no caching of script
  //var SCRIPT_URL = Script.resolvePath("myEntityScript.js")

  var myEntity = Entities.addEntity({
      name: "Cusack_Testing",
      type: "Shape",
      shapeType: "cylinder-y",
      color: {
          red: 200,
          green: 10,
          blue: 200
      },
      position: center,
      dimensions: {
          x: 2,
          y: 4,
          z: 2
      },
      lifetime: -1,
      //script: SCRIPT_URL
  })


  function cleanup() {
      // Entities.deleteEntity(myEntity);
  }

  Script.scriptEnding.connect(cleanup);