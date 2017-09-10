var Status = function (a) {
  var that  = this;
  this.ros = a.ros;
  this.divID = a.divID || 'status';
  this.div = document.getElementById(this.divID);
  this.botVar = {
    maxPower : a.botMaxPower || 165.0,
    topicName : a.botTopicName || '/mobile_base/sensors/core',
    topicType : a.botTopicType || 'kobuki_msgs/SensorState'
  };
  this.lapVar = {
    topicName : a.laptopTopicName || '/laptop_charge',
    topicType : a.laptopTopicType || 'sensor_msgs/BatteryState'
  }
  this.updateInterInit = a.updateInterInit || 300000;
  this.color = a.color || {
    good : 'rgba(0,184,222,0.7)',
    warning : 'rgba(255,160,0,0.7)',
    danger : 'rgba(255,24,0,0.7)'
  };
  this.botBattery = new ROSLIB.Topic({
    ros : that.ros,
    name : that.botVar.topicName,
    messageType : that.botVar.topicType
  });
  this.laptopBattery = new ROSLIB.Topic({
    ros : that.ros,
    name : that.lapVar.topicName,
    messageType : that.lapVar.topicType
  });
  this.updateInter = this.updateInterInit;
  this.init = function () {
    var para = document.createElement('p');
    para.setAttribute('class','header');
    para.appendChild(document.createTextNode('Status'));
    that.div.appendChild(para);
    var ul = document.createElement('ul');
    ul.setAttribute('class','progress');
    that.lapBat = that.createStatusProgresLi("Laptop battery",'laptopBat');
    that.botBat = that.createStatusProgresLi("Bot battery",'botBat');
    ul.appendChild(that.lapBat);
    ul.appendChild(that.botBat);
    that.div.appendChild(ul);
    that.div.innerHTML += "";
    that.updateBattery();
  };
  this.createStatusProgresLi = function (name,id) {
    var li = document.createElement('li');
    li.setAttribute('data-name',name);
    li.setAttribute('data-percent','?%');
    li.setAttribute('id',id);
    var svg = document.createElement('svg');
    svg.setAttribute('viewBox','-10 -10 220 220');
    var g = document.createElementNS('http://www.w3.org/2000/svg','g');
    g.setAttribute('fill','none');
    g.setAttribute('stroke-width',10);
    g.setAttribute('transform','translate(100,100)');
    g.setAttribute('stroke',that.color.warning);
    g.appendChild(that.createPath('M 0,-100 A 100,100 0 0,1 86.6,-50'));
    g.appendChild(that.createPath('M 86.6,-50 A 100,100 0 0,1 86.6,50'));
    g.appendChild(that.createPath('M 86.6,50 A 100,100 0 0,1 0,100'));
    g.appendChild(that.createPath('M 0,100 A 100,100 0 0,1 -86.6,50'));
    g.appendChild(that.createPath('M -86.6,50 A 100,100 0 0,1 -86.6,-50'));
    g.appendChild(that.createPath('M -86.6,-50 A 100,100 0 0,1 0,-100'));
    svg.appendChild(g);
    li.appendChild(svg);
    svg = document.createElement('svg');
    svg.setAttribute('viewBox','-10 -10 220 220');
    svg.appendChild(that.createPath('M200,100 C200,44.771525 155.228475,0 100,0 C44.771525,0 0,44.771525 0,100 C0,155.228475 44.771525,200 100,200 C155.228475,200 200,155.228475 200,100 Z',629));
    li.appendChild(svg);
    return li;
  };
  this.createPath = function (d,dashoffset) {
    var path = document.createElementNS('http://www.w3.org/2000/svg','path');
    path.setAttribute('d',d);
    if (dashoffset) {
      path.setAttribute('stroke-dashoffset',dashoffset);
    }
    return path;
  }
  this.updateBattery = function () {
    that.botBattery.subscribe(function(sensor_msgs) {
      var botBat = document.getElementById('botBat');
      var percent = (sensor_msgs.battery/that.botVar.maxPower*100.0).toFixed(1);
      if (percent < 25) {
        botBat.children[0].children[0].setAttribute('stroke',that.color.danger);
      } else if (percent < 50) {
        botBat.children[0].children[0].setAttribute('stroke',that.color.warning);
      } else {
        botBat.children[0].children[0].setAttribute('stroke',that.color.good);
      }
      botBat.dataset.percent = percent+"%";
      botBat.children[1].children[0].setAttribute('stroke-dashoffset',percent*6.29);
      that.updateInter = that.updateInterInit * (percent/100.0);
      that.botBattery.unsubscribe();
    });
    that.laptopBattery.subscribe(function(sensor_msgs) {
      var lapBat = document.getElementById('laptopBat');
      if (sensor_msgs.percentage < 25) {
        lapBat.children[0].children[0].setAttribute('stroke',that.color.danger);
      } else if (sensor_msgs.percentage < 50) {
        lapBat.children[0].children[0].setAttribute('stroke',that.color.warning);
      } else {
        lapBat.children[0].children[0].setAttribute('stroke',that.color.good);
      }
      if (that.updateInterInit * (sensor_msgs.percentage/100.0) > that.updateInter) {
        that.updateInter = that.updateInterInit * (sensor_msgs.percentage/100.0)
      }
      lapBat.dataset.percent = sensor_msgs.percentage + "%";
      lapBat.children[1].children[0].setAttribute('stroke-dashoffset',sensor_msgs.percentage*6.29);
      that.laptopBattery.unsubscribe();
    });
    setTimeout(that.updateBattery,that.updateInter);
  };
  this.init();
};
