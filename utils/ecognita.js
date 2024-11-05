/* EcognitaWeb3D */
export const EcognitaWeb3D = {};

EcognitaWeb3D.Filter = {
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

EcognitaWeb3D.RenderPipeLine = {
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

// FilterViewer
// WebGLEnv

/* EcognitaMathLib */
export const EcognitaMathLib = {
  imread: function (file) {
    var img = new Image();
    return (img.src = file), img;
  },

  HSV2RGB: function (h, s, v, a) {
    if (!(s > 1 || v > 1 || a > 1)) {
      var th = h % 360,
        i = Math.floor(th / 60),
        f = th / 60 - i,
        m = v * (1 - s),
        n = v * (1 - s * f),
        k = v * (1 - s * (1 - f)),
        color = new Array();
      if (s > 0 || s < 0) {
        var r = new Array(v, n, m, m, k, v),
          g = new Array(k, v, v, n, m, m),
          b = new Array(m, m, k, v, v, n);
        color.push(r[i], g[i], b[i], a);
      } else color.push(v, v, v, a);
      return color;
    }
  },
};

WebGLMatrix = {};
WebGLMatrix.prototype.create = function () {
  return new Float32Array(16);
};
WebGLMatrix.prototype.identity = function (mat) {
  return (
    (mat[0] = 1),
    (mat[1] = 0),
    (mat[2] = 0),
    (mat[3] = 0),
    (mat[4] = 0),
    (mat[5] = 1),
    (mat[6] = 0),
    (mat[7] = 0),
    (mat[8] = 0),
    (mat[9] = 0),
    (mat[10] = 1),
    (mat[11] = 0),
    (mat[12] = 0),
    (mat[13] = 0),
    (mat[14] = 0),
    (mat[15] = 1),
    mat
  );
};
WebGLMatrix.prototype.multiply = function (mat1, mat2) {
  var mat = this.create(),
    a = mat1[0],
    b = mat1[1],
    c = mat1[2],
    d = mat1[3],
    e = mat1[4],
    f = mat1[5],
    g = mat1[6],
    h = mat1[7],
    i = mat1[8],
    j = mat1[9],
    k = mat1[10],
    l = mat1[11],
    m = mat1[12],
    n = mat1[13],
    o = mat1[14],
    p = mat1[15],
    A = mat2[0],
    B = mat2[1],
    C = mat2[2],
    D = mat2[3],
    E = mat2[4],
    F = mat2[5],
    G = mat2[6],
    H = mat2[7],
    I = mat2[8],
    J = mat2[9],
    K = mat2[10],
    L = mat2[11],
    M = mat2[12],
    N = mat2[13],
    O = mat2[14],
    P = mat2[15];
  return (
    (mat[0] = A * a + B * e + C * i + D * m),
    (mat[1] = A * b + B * f + C * j + D * n),
    (mat[2] = A * c + B * g + C * k + D * o),
    (mat[3] = A * d + B * h + C * l + D * p),
    (mat[4] = E * a + F * e + G * i + H * m),
    (mat[5] = E * b + F * f + G * j + H * n),
    (mat[6] = E * c + F * g + G * k + H * o),
    (mat[7] = E * d + F * h + G * l + H * p),
    (mat[8] = I * a + J * e + K * i + L * m),
    (mat[9] = I * b + J * f + K * j + L * n),
    (mat[10] = I * c + J * g + K * k + L * o),
    (mat[11] = I * d + J * h + K * l + L * p),
    (mat[12] = M * a + N * e + O * i + P * m),
    (mat[13] = M * b + N * f + O * j + P * n),
    (mat[14] = M * c + N * g + O * k + P * o),
    (mat[15] = M * d + N * h + O * l + P * p),
    mat
  );
};
WebGLMatrix.prototype.scale = function (mat1, param_scale) {
  var mat = this.create();
  if (3 == param_scale.length)
    return (
      (mat[0] = mat1[0] * param_scale[0]),
      (mat[1] = mat1[1] * param_scale[0]),
      (mat[2] = mat1[2] * param_scale[0]),
      (mat[3] = mat1[3] * param_scale[0]),
      (mat[4] = mat1[4] * param_scale[1]),
      (mat[5] = mat1[5] * param_scale[1]),
      (mat[6] = mat1[6] * param_scale[1]),
      (mat[7] = mat1[7] * param_scale[1]),
      (mat[8] = mat1[8] * param_scale[2]),
      (mat[9] = mat1[9] * param_scale[2]),
      (mat[10] = mat1[10] * param_scale[2]),
      (mat[11] = mat1[11] * param_scale[2]),
      (mat[12] = mat1[12]),
      (mat[13] = mat1[13]),
      (mat[14] = mat1[14]),
      (mat[15] = mat1[15]),
      mat
    );
};
WebGLMatrix.prototype.translate = function (mat1, param_translate) {
  var mat = this.create();
  if (3 == param_translate.length)
    return (
      (mat[0] = mat1[0]),
      (mat[1] = mat1[1]),
      (mat[2] = mat1[2]),
      (mat[3] = mat1[3]),
      (mat[4] = mat1[4]),
      (mat[5] = mat1[5]),
      (mat[6] = mat1[6]),
      (mat[7] = mat1[7]),
      (mat[8] = mat1[8]),
      (mat[9] = mat1[9]),
      (mat[10] = mat1[10]),
      (mat[11] = mat1[11]),
      (mat[12] = mat1[0] * param_translate[0] + mat1[4] * param_translate[1] + mat1[8] * param_translate[2] + mat1[12]),
      (mat[13] = mat1[1] * param_translate[0] + mat1[5] * param_translate[1] + mat1[9] * param_translate[2] + mat1[13]),
      (mat[14] =
        mat1[2] * param_translate[0] + mat1[6] * param_translate[1] + mat1[10] * param_translate[2] + mat1[14]),
      (mat[15] =
        mat1[3] * param_translate[0] + mat1[7] * param_translate[1] + mat1[11] * param_translate[2] + mat1[15]),
      mat
    );
};
WebGLMatrix.prototype.rotate = function (mat1, angle, axis) {
  var mat = this.create();
  if (3 == axis.length) {
    var sq = Math.sqrt(axis[0] * axis[0] + axis[1] * axis[1] + axis[2] * axis[2]);
    if (sq) {
      var a = axis[0],
        b = axis[1],
        c = axis[2];
      1 != sq && ((a *= sq = 1 / sq), (b *= sq), (c *= sq));
      var d = Math.sin(angle),
        e = Math.cos(angle),
        f = 1 - e,
        g = mat1[0],
        h = mat1[1],
        i = mat1[2],
        j = mat1[3],
        k = mat1[4],
        l = mat1[5],
        m = mat1[6],
        n = mat1[7],
        o = mat1[8],
        p = mat1[9],
        q = mat1[10],
        r = mat1[11],
        s = a * a * f + e,
        t = b * a * f + c * d,
        u = c * a * f - b * d,
        v = a * b * f - c * d,
        w = b * b * f + e,
        x = c * b * f + a * d,
        y = a * c * f + b * d,
        z = b * c * f - a * d,
        A = c * c * f + e;
      return (
        angle
          ? mat1 != mat && ((mat[12] = mat1[12]), (mat[13] = mat1[13]), (mat[14] = mat1[14]), (mat[15] = mat1[15]))
          : (mat = mat1),
        (mat[0] = g * s + k * t + o * u),
        (mat[1] = h * s + l * t + p * u),
        (mat[2] = i * s + m * t + q * u),
        (mat[3] = j * s + n * t + r * u),
        (mat[4] = g * v + k * w + o * x),
        (mat[5] = h * v + l * w + p * x),
        (mat[6] = i * v + m * w + q * x),
        (mat[7] = j * v + n * w + r * x),
        (mat[8] = g * y + k * z + o * A),
        (mat[9] = h * y + l * z + p * A),
        (mat[10] = i * y + m * z + q * A),
        (mat[11] = j * y + n * z + r * A),
        mat
      );
    }
  }
};
WebGLMatrix.prototype.viewMatrix = function (cam, target, up) {
  var mat = this.identity(this.create());
  if (3 == cam.length && 3 == target.length && 3 == up.length) {
    var camX = cam[0],
      camY = cam[1],
      camZ = cam[2],
      targetX = target[0],
      targetY = target[1],
      targetZ = target[2],
      upX = up[0],
      upY = up[1],
      upZ = up[2];
    if (camX == targetX && camY == targetY && camZ == targetZ) return mat;
    var forwardX = camX - targetX,
      forwardY = camY - targetY,
      forwardZ = camZ - targetZ,
      l = 1 / Math.sqrt(forwardX * forwardX + forwardY * forwardY + forwardZ * forwardZ),
      rightX = upY * (forwardZ *= l) - upZ * (forwardY *= l),
      rightY = upZ * (forwardX *= l) - upX * forwardZ,
      rightZ = upX * forwardY - upY * forwardX;
    return (
      (l = Math.sqrt(rightX * rightX + rightY * rightY + rightZ * rightZ))
        ? ((rightX *= l = 1 / Math.sqrt(rightX * rightX + rightY * rightY + rightZ * rightZ)),
          (rightY *= l),
          (rightZ *= l))
        : ((rightX = 0), (rightY = 0), (rightZ = 0)),
      (upX = forwardY * rightZ - forwardZ * rightY),
      (upY = forwardZ * rightX - forwardX * rightZ),
      (upZ = forwardX * rightY - forwardY * rightX),
      (mat[0] = rightX),
      (mat[1] = upX),
      (mat[2] = forwardX),
      (mat[3] = 0),
      (mat[4] = rightY),
      (mat[5] = upY),
      (mat[6] = forwardY),
      (mat[7] = 0),
      (mat[8] = rightZ),
      (mat[9] = upZ),
      (mat[10] = forwardZ),
      (mat[11] = 0),
      (mat[12] = -(rightX * camX + rightY * camY + rightZ * camZ)),
      (mat[13] = -(upX * camX + upY * camY + upZ * camZ)),
      (mat[14] = -(forwardX * camX + forwardY * camY + forwardZ * camZ)),
      (mat[15] = 1),
      mat
    );
  }
};
WebGLMatrix.prototype.perspectiveMatrix = function (fovy, aspect, near, far) {
  var mat = this.identity(this.create()),
    t = near * Math.tan((fovy * Math.PI) / 360),
    r,
    a = 2 * (t * aspect),
    b = 2 * t,
    c = far - near;
  return (
    (mat[0] = (2 * near) / a),
    (mat[1] = 0),
    (mat[2] = 0),
    (mat[3] = 0),
    (mat[4] = 0),
    (mat[5] = (2 * near) / b),
    (mat[6] = 0),
    (mat[7] = 0),
    (mat[8] = 0),
    (mat[9] = 0),
    (mat[10] = -(far + near) / c),
    (mat[11] = -1),
    (mat[12] = 0),
    (mat[13] = 0),
    (mat[14] = (-far * near * 2) / c),
    (mat[15] = 0),
    mat
  );
};
WebGLMatrix.prototype.orthoMatrix = function (left, right, top, bottom, near, far) {
  var mat = this.identity(this.create()),
    h = right - left,
    v = top - bottom,
    d = far - near;
  return (
    (mat[0] = 2 / h),
    (mat[1] = 0),
    (mat[2] = 0),
    (mat[3] = 0),
    (mat[4] = 0),
    (mat[5] = 2 / v),
    (mat[6] = 0),
    (mat[7] = 0),
    (mat[8] = 0),
    (mat[9] = 0),
    (mat[10] = -2 / d),
    (mat[11] = 0),
    (mat[12] = -(left + right) / h),
    (mat[13] = -(top + bottom) / v),
    (mat[14] = -(far + near) / d),
    (mat[15] = 1),
    mat
  );
};
WebGLMatrix.prototype.transpose = function (mat1) {
  var mat = this.create();
  return (
    (mat[0] = mat1[0]),
    (mat[1] = mat1[4]),
    (mat[2] = mat1[8]),
    (mat[3] = mat1[12]),
    (mat[4] = mat1[1]),
    (mat[5] = mat1[5]),
    (mat[6] = mat1[9]),
    (mat[7] = mat1[13]),
    (mat[8] = mat1[2]),
    (mat[9] = mat1[6]),
    (mat[10] = mat1[10]),
    (mat[11] = mat1[14]),
    (mat[12] = mat1[3]),
    (mat[13] = mat1[7]),
    (mat[14] = mat1[11]),
    (mat[15] = mat1[15]),
    mat
  );
};
EcognitaMathLib.WebGLMatrix = WebGLMatrix;
