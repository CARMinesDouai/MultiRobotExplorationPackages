var Viewer = function (a) {
  var that  = this;
  this.map =  a.map;
  this.vmap = a.vmap;
  this.camera = a.camera;
  this.currentView = this.map.divID;
  this.changeView = function (event) {
    document.getElementById('indicator').style.width = event.clientWidth+'px';
    document.getElementById('indicator').style.left = event.offsetLeft+'px';
    var v = document.getElementById('viewer').children;
    for (var i=0; i<v.length; i++) {
      if (v[i].id != event.innerHTML) {
        v[i].style.display = 'none';
        that[v[i].id].unsubscribe();
      }
    }
    that.currentView = event.innerHTML;
    document.getElementById(that.currentView).style.display = 'block';
    that[that.currentView].subscribe();
  };
  this.zoom = function (scale) {
    if (that.currentView == that.map.divID) {
      that.map.zoom(scale);
    } else if (that.currentView == that.vmap.divID) {
      that.vmap.zoom(scale);
    }
  };
  this.scroll = function (ori,step) {
    var doc = document.getElementById(that.currentView);
    if (doc.scrollLeftMax == 0 && doc.scrollTopMax == 0) {
      return;
    }
    var pas = step || 50;
    if (ori == 'up') {
      doc.scrollTop -= pas;
    } else if (ori == 'left') {
      doc.scrollLeft -= pas;
    } else if (ori == 'right') {
      doc.scrollLeft += pas;
    } else if (ori == 'down') {
      doc.scrollTop += pas;
    } else {
      console.log('invalid orientation');
    }
  };
  this.setPose = function () {
    if (that.currentView == that.map.divID) {
      that.map.settingPose = !that.map.settingPose;
    } else if (that.currentView == that.vmap.divID) {
      that.vmap.settingPose = !that.vmap.settingPose;
    }
  };
};
