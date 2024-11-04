const Filter = {
  0: "LAPLACIAN",
  1: "SOBEL",
  2: "GAUSSIAN",
  6: "ANISTROPIC",
  8: "NOISELIC",
  9: "DoG",
  10: "XDoG",
  12: "FXDoG",
  ANISTROPIC: 6,
  DoG: 9,
  FXDoG: 12,
  GAUSSIAN: 2,
  LAPLACIAN: 0,
  NOISELIC: 8,
  SOBEL: 1,
  XDoG: 10,
};

const RenderPipeLine = {
  0: "CONVOLUTION_FILTER",
  1: "ANISTROPIC",
  2: "BLOOM_EFFECT",
  3: "CONVOLUTION_TWICE",
  4: "ABSTRACTION",
  ABSTRACTION: 4,
  ANISTROPIC: 1,
  BLOOM_EFFECT: 2,
  CONVOLUTION_FILTER: 0,
  CONVOLUTION_TWICE: 3,
};

class FilterViewer {
  constructor(cvs) {
    super(cvs);
    this.alpha = 50;
    this.beta = 50;
    this.gamma = 0.3; // 注意这里将30改为0.3，因为gamma值通常在0到1之间
    this.light = 100;
    this.clean = 0;
    this.pixels = 1000; // 将1e3转换为1000，因为1e3等于1000
    this.isoutline = true;
    this.imgbase64 = null; // 初始化为null，因为没有提供具体的值
  }

  loadAssets() {
    this.loadTexture("./image/k0.png", false);
    this.loadTexture("./image/visual_rgb.png");
    this.loadTexture("./image/lion.png", false);
    this.loadTexture("./image/cat.jpg", false);
    this.loadTexture("./image/noise.png", false);
  }

  getReqQuery() {
    if (1 == window.location.href.split("?").length) return {};
    var queryString = window.location.href.split("?")[1],
      queryObj = {};
    if ("" != queryString)
      for (var querys = queryString.split("&"), i = 0; i < querys.length; i++) {
        var key = querys[i].split("=")[0],
          value = querys[i].split("=")[1];
        queryObj[key] = value;
      }
    return queryObj;
  }
}

const WebGLEnv = {};

WebGLEnv.prototype.chkWebGLEnv = function (cvs) {
  this.canvas = cvs;
  try {
    gl =
      this.canvas.getContext("webgl", {
        preserveDrawingBuffer: !0,
      }) || this.canvas.getContext("experimental-webgl");
  } catch (e) {}
  if (!gl) throw new Error("Could not initialise WebGL");
  var ext;
  if (null == gl.getExtension("OES_texture_float")) throw new Error("float texture not supported");
};

WebGLEnv.prototype.initGlobalVariables = function () {
  (this.vbo = new Array()),
    (this.ibo = new Array()),
    (this.Texture = new Utils.HashSet()),
    (this.matUtil = new EcognitaMathLib.WebGLMatrix()),
    (this.quatUtil = new EcognitaMathLib.WebGLQuaternion());
};

WebGLEnv.prototype.initGlobalMatrix = function () {
  this.MATRIX = new Utils.HashSet();
  var m = this.matUtil;
  this.MATRIX.set("mMatrix", m.identity(m.create())),
    this.MATRIX.set("vMatrix", m.identity(m.create())),
    this.MATRIX.set("pMatrix", m.identity(m.create())),
    this.MATRIX.set("vpMatrix", m.identity(m.create())),
    this.MATRIX.set("mvpMatrix", m.identity(m.create())),
    this.MATRIX.set("invMatrix", m.identity(m.create()));
};

class EcognitaWeb3D {}
