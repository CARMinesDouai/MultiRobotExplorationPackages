var Vmap = function (a) {
  var that  = this;
  this.ros  = a.ros;
  this.continuous = a.continuous || false;
  this.divID = a.divID || 'vmap';
  this.showPath = a.showPath || false;
  this.settingPose = false;
  this.vmapServerName = a.vmapServerName || '/vmap';
  this.utils =  { rosQuaternionToGlobalTheta : function(orientation) {
                    var q0 = orientation.w;
                    var q1 = orientation.x;
                    var q2 = orientation.y;
                    var q3 = orientation.z;
                    return -Math.atan2(2 * (q0 * q3 + q1 * q2), 1 - 2 * (q2 * q2 + q3 * q3)) * 180.0 / Math.PI;},
                  globalToRos : function (x,y) {
                    var transform = 1.0/that.transMatrix[0];
                    return {x : (((x-that.transMatrix[4])*transform)-that.decaX)/that.scale,
                            y : -(((y-that.transMatrix[5])*transform)-that.decaY)/that.scale };
                    },
                  rosToGlobal : function (x,y) {
                    return {x : (that.decaX + (x*that.scale)),
                            y : (that.decaY - (y*that.scale))};
                  }
                  };
  this.eventsVar =  { position : null,
                      positionVec3 : null,
                      thetaRadians : 0,
                      thetaDegrees : 0,
                      orientationMarker : null,
                      mouseDown : false,
                      xDelta : 0,
                      yDelta : 0,
                      lastTimeStamp : 0,
                      pathMarker : null};
  this.serverName = a.serverName || '/move_base';
  this.actionName = a.actionName || 'move_base_msgs/MoveBaseAction';
  this.navServerName = a.navServerName || '/move_base/NavfnROS/plan';
  this.navActionName = a.navActionName || 'nav_msgs/Path';
  this.actionClient = new ROSLIB.ActionClient({
    ros : that.ros,
    actionName : that.actionName,
    serverName : that.serverName
  });
  this.transMatrix = [1,0,0,1,0,0];
  this.subscribe = {};
  this.subscriber = {
    botPos : function () {
      that.subscribe.botPos.subscribe(function(pos) {
        var botSVGPos = document.getElementById('bot');
        if (botSVGPos === null) {
          return;
        }
        that.botPos = that.utils.rosToGlobal(pos.position.x,pos.position.y);
        var rotation = 165 + that.utils.rosQuaternionToGlobalTheta(pos.orientation);
        botSVGPos.setAttribute('points',(that.botPos.x - 7)+","+(that.botPos.y - 2.21)+
                          " "+(that.botPos.x + 3)+","+(that.botPos.y - 2.21)+
                          " "+(that.botPos.x + 1.76)+","+(that.botPos.y + 2.21));
        botSVGPos.setAttribute('style',"fill:blue;stroke:blue;stroke-width:1;transform-origin:"+
                          that.botPos.x+"px "+that.botPos.y+"px;transform:rotate("+rotation+"deg)");
      });
    },
    map : function () {
      that.subscribe.map.subscribe(function(message) {
        that.svg.innerHTML=that.toSVG(message);
        if (!that.continuous) {
          that.subscribe.map.unsubscribe();
        }
      });
    }
  };
  this.init = function () {
    document.getElementById(that.divID).innerHTML =
    "<svg id='svg_vmap' transform='matrix(1 0 0 1 0 0)' height='0' width='0'></svg>";
    that.svg = document.getElementById('svg_vmap');
    that.initMeta();
    that.botPos = {x : that.decaX, y : that.decaY};
    that.subscribe.botPos = new ROSLIB.Topic({
      ros : that.ros,
      name : '/robot_pose',
      messageType : 'geometry_msgs/Pose'
    });
    that.subscribe.map = new ROSLIB.Topic({
      ros : that.ros,
      name : that.vmapServerName,
      messageType : 'torob_msgs/VectorMap'
    });
    that.subscriber.botPos();
    that.subscriber.map();
    that.navPlan = new ROSLIB.Topic({
      ros : that.ros,
      name : that.navServerName,
      messageType : that.navActionName
    });
    that.svg.onmousedown = function (event) {
      if (event.buttons == 1) {
        that.mouseEventHandler(event,'down');
      }
    };
    that.svg.onmousemove = function(event) {
      if (event.buttons == 1) {
        that.mouseEventHandler(event,'move');
      }
    };
    that.svg.onmouseup = function(event) {
        that.mouseEventHandler(event,'up');
   };
  };
  this.initMeta = function () {
    var map_metadata = new ROSLIB.Topic({
      ros : that.ros,
      name : '/map_metadata',
      messageType : 'nav_msgs/MapMetaData'
    });
    map_metadata.subscribe(function(meta) {
      that.meta = meta;
      that.resolution = meta.resolution.toFixed(8);
      that.scale = 1/that.resolution;
      that.width = meta.width;
      that.height = meta.height;
      that.decaX = (that.width/2.0);
      that.decaY = (that.height/2.0);
      that.svg.setAttributeNS(null, "width", meta.width);
      that.svg.setAttributeNS(null, "height", meta.height);
      map_metadata.unsubscribe();
    });
  };
  this.toSVG =  function (matrix) {
    that.initMeta();
    var SVG = "";
    var edges = matrix.edges;
    var nodes = matrix.nodes;
    SVG += " <circle cx='"+that.decaX+"' cy='"+that.decaY+"' r='1' stroke='black' stroke-width='3' />";
    SVG += "<polygon id='bot'></polygon>"
    for (var i = 0; i < edges.length; i++) {
      SVG += that.edgeToSVG(   i,
        {
          src : nodes[edges[i].src],
          trg : nodes[edges[i].trg],
          type : edges[i].type,
          weight : edges[i].weight
        });
      }
      return SVG;
  };
  this.edgeToSVG = function(id, edge) {
    var color = "black";
    if (edge.src.type == 2) {
      color = "green";
    }
    var line = "<line ";

    if (id !== null) {
      line += "id='" + id + "' ";
    }
    var src = that.utils.rosToGlobal(edge.src.x,edge.src.y);
    var trg = that.utils.rosToGlobal(edge.trg.x,edge.trg.y);
    line += "x1='"+src.x+"' y1='"+src.y+"' "+"x2='"+trg.x+"' y2='"+trg.y+"' "
    +"style='stroke:"+color+";stroke-width:2' />";
    return line;
  };
  this.sendGoal = function(pose) {
    if (that.showPath) {
      that.navPlan.subscribe(function(path) {
        if (that.eventsVar.pathMarker === null) {
          that.eventsVar.pathMarker = document.createElementNS('http://www.w3.org/2000/svg','polyline');
          that.eventsVar.pathMarker.setAttribute('style',"stroke:red;stroke-width:1;fill:none;");
          that.svg.appendChild(that.eventsVar.pathMarker);
        }
        var points = "";
        for (pathPose of path.poses) {
          var posTmp = that.utils.rosToGlobal(pathPose.pose.position.x,pathPose.pose.position.y);
          points += posTmp.x+","+posTmp.y+" ";
        }
        that.eventsVar.pathMarker.setAttribute('points', points);
        that.navPlan.unsubscribe();
      });
    }
    // create a goal
    var goal = new ROSLIB.Goal({
      actionClient : that.actionClient,
      goalMessage : {
        target_pose : {
          header : {
            frame_id : '/map'
          },
          pose : pose
        }
      }
    });
    goal.send();
    var posTmp = that.utils.rosToGlobal(pose.position.x,pose.position.y);
    var rotation = 165 + that.utils.rosQuaternionToGlobalTheta(pose.orientation);
    var goalMarker = document.createElementNS('http://www.w3.org/2000/svg','polygon');
    goalMarker.setAttribute('points',(posTmp.x - 7)+","+(posTmp.y - 2.21)+
                      " "+(posTmp.x + 3)+","+(posTmp.y - 2.21)+
                      " "+(posTmp.x + 1.76)+","+(posTmp.y + 2.21));
    goalMarker.setAttribute('style',"fill:#39bda7;stroke:#39bda7;stroke-width:1;transform-origin:"+
    posTmp.x+"px "+posTmp.y+"px;transform:rotate("+rotation+"deg)");
    that.svg.appendChild(goalMarker);
    goal.on('result', function() {
      that.svg.removeChild(goalMarker);
      that.eventsVar.pathMarker.setAttribute('points', "");
    });
  };
  this.setPose = function (pose) {
    var cmdVel = new ROSLIB.Topic({
      ros : ros,
      name : '/initialpose',
      messageType : 'geometry_msgs/PoseWithCovarianceStamped'
    });
    var currentTime = new Date();
    var secs = Math.floor(currentTime.getTime()/1000);
    var nsecs = Math.round(1000000000*(currentTime.getTime()/1000-secs));
    var initialpose = new ROSLIB.Message({
      header: {
        seq: 0,
        stamp: {
          secs: secs,
          nsecs: nsecs
        },
        frame_id: 'map'
      },
      pose : {pose : pose,
              covariance: [0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.25, 0.0, 0.0, 0.0, 0.0,
                   0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                   0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.06853891945200942]
             }
    });
    cmdVel.publish(initialpose);
    that.settingPose = false;
  };
  this.mouseEventHandler = function(event, mouseState) {
    event.preventDefault();
      if (mouseState === 'down'){
        if (event.target.id == "svg_vmap") {
          that.eventsVar.position = that.utils.globalToRos(event.layerX, event.layerY);
        } else {
          var point = event.target.getAttribute('points').split(" ")[0].split(",");
          point = {x : parseInt(point[0])+event.layerX, y : parseInt(point[1])+event.layerY};
          that.eventsVar.position = that.utils.globalToRos(point.x, point.y);
        }
        that.eventsVar.positionVec3 = new ROSLIB.Vector3(that.eventsVar.position);
        that.eventsVar.mouseDown = true;
      }
      else if (mouseState === 'move'){
        if (event.timeStamp - that.eventsVar.lastTimeStamp < 25) {
          return;
        } else {
          that.eventsVar.lastTimeStamp = event.timeStamp;
        }
        if ( that.eventsVar.mouseDown === true) {
          if (event.target.id == "svg_vmap") {
            var currentPos = that.utils.globalToRos(event.layerX, event.layerY);
          } else {
            var point = event.target.getAttribute('points').split(" ")[0].split(",");
            point = {x : parseInt(point[0])+event.layerX, y : parseInt(point[1])+event.layerY};
            var currentPos = that.utils.globalToRos(point.x, point.y);
          }
          var currentPosVec3 = new ROSLIB.Vector3(currentPos);

          if (that.eventsVar.orientationMarker === null) {
            that.eventsVar.orientationMarker = document.createElementNS('http://www.w3.org/2000/svg','polygon');
            that.svg.appendChild(that.eventsVar.orientationMarker);
          }
          that.eventsVar.xDelta =  currentPosVec3.x - that.eventsVar.positionVec3.x;
          that.eventsVar.yDelta =  currentPosVec3.y - that.eventsVar.positionVec3.y;
          that.eventsVar.thetaRadians  = Math.atan2(that.eventsVar.xDelta,that.eventsVar.yDelta);
          if (that.eventsVar.thetaRadians >= 0 && that.eventsVar.thetaRadians <= Math.PI) {
            that.eventsVar.thetaRadians += (3 * Math.PI / 2);
          } else {
            that.eventsVar.thetaRadians -= (Math.PI/2);
          }
          that.eventsVar.thetaDegrees = that.eventsVar.thetaRadians * (180.0 / Math.PI);
          var posTmp = {x : (that.decaX + (that.eventsVar.positionVec3.x*that.scale)),
            y : (that.decaY -(that.eventsVar.positionVec3.y*that.scale))};

          that.eventsVar.orientationMarker.setAttribute('points',(posTmp.x - 7)+","+(posTmp.y - 2.21)+
                            " "+(posTmp.x + 3)+","+(posTmp.y - 2.21)+
                            " "+(posTmp.x + 1.76)+","+(posTmp.y + 2.21));
          that.eventsVar.orientationMarker.setAttribute('style',"fill:red;stroke:red;stroke-width:1;transform-origin:"+
          posTmp.x+"px "+posTmp.y+"px;transform:rotate("+(165+that.eventsVar.thetaDegrees)+"deg)");
        }
      } else if (that.eventsVar.mouseDown) {
        that.eventsVar.mouseDown = false;
        if (that.eventsVar.orientationMarker !== null) {
          that.svg.removeChild(that.eventsVar.orientationMarker);
          that.eventsVar.orientationMarker = null;
        }
        if (event.target.id == "svg_vmap") {
          var goalPos = that.utils.globalToRos(event.layerX, event.layerY);
        } else {
          var point = event.target.getAttribute('points').split(" ")[0].split(",");
          point = {x : parseInt(point[0])+event.layerX, y : parseInt(point[1])+event.layerY};
          var goalPos = that.utils.globalToRos(point.x, point.y);
        }
        var goalPosVec3 = new ROSLIB.Vector3(goalPos);

        that.eventsVar.xDelta =  goalPosVec3.x - that.eventsVar.positionVec3.x;
        that.eventsVar.yDelta =  goalPosVec3.y - that.eventsVar.positionVec3.y;

        that.eventsVar.thetaRadians  = Math.atan2(that.eventsVar.xDelta,that.eventsVar.yDelta);

        if (that.eventsVar.thetaRadians >= 0 && that.eventsVar.thetaRadians <= Math.PI) {
          that.eventsVar.thetaRadians += (3 * Math.PI / 2);
        } else {
          that.eventsVar.thetaRadians -= (Math.PI/2);
        }

        var qz =  Math.sin(-that.eventsVar.thetaRadians/2.0);
        var qw =  Math.cos(-that.eventsVar.thetaRadians/2.0);

        var orientation = new ROSLIB.Quaternion({x:0, y:0, z:qz, w:qw});

        var pose = new ROSLIB.Pose({
          position :    that.eventsVar.positionVec3,
          orientation : orientation
        });
        if (that.settingPose) {
          that.setPose(pose);
        } else {
          that.sendGoal(pose);
        }
      }
    };
  this.zoom = function (scale) {
    if (that.transMatrix[0] <= 0.25 && (scale < 1)) {
      return;
    }
    that.transMatrix[0] = that.transMatrix[3] += (scale-1);
    that.transMatrix[1] *= scale;
    that.transMatrix[2] *= scale;
    that.transMatrix[4] += (1-scale)*that.width/4;
    that.transMatrix[5] += (1-scale)*that.height/4;
    if (that.transMatrix[0] > 1) {
      that.svg.setAttributeNS(null, "width", that.width*that.transMatrix[0]);
      that.svg.setAttributeNS(null, "height", that.height*that.transMatrix[0]);
    } else {
      that.svg.setAttributeNS(null, "width", that.width);
      that.svg.setAttributeNS(null, "height", that.height);
    }
    var newMatrix = "matrix(" +  that.transMatrix.join(' ') + ")";
    that.svg.setAttributeNS(null, "transform", newMatrix);
  };
  this.unsubscribe = function () {
    that.subscribe.botPos.unsubscribe();
    that.subscribe.map.unsubscribe();
  };
  this.subscribe = function () {
    that.subscriber.botPos();
    that.subscriber.map();
  };
  this.init();
};
  // arg : torob_msgs/VectorMap
  // var toto = document.getElementById("1");
  // toto.setAttribute("x1",50);
  // edgeToSVG(2,{src : {y:2.570664882659912,x:7.478055477142334,type:2,weight:0},trg : {y:1.7749994993209839,x:7.924130439758301,type:2,weight:0},type : 0,weight : 0})
