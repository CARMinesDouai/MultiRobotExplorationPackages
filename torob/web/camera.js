var Camera = function (a) {
  var that = this;
  var a = a || {};
  this.divID = a.divID || 'camera';
  this.unsubscribe = function () {
    if (that.viewer == null) {
      return;
    }
    document.getElementById(that.divID).removeChild(that.viewer);
    that.viewer == null;
  };
  this.subscribe = function () {
    if (that.viewer == null) {
      that.viewer = document.createElement('img');
      that.viewer.setAttribute('src',"http://"+location.hostname+":8081/stream?topic=/camera/rgb/image_color");
    }
    document.getElementById(that.divID).appendChild(that.viewer);
  };
}
//<img src="http://localhost:8081/stream?topic=/camera/rgb/image_color" alt="http://localhost:8081/stream?topic=/camera/rgb/image_color">
