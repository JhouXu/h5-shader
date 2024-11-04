// Math Library
class EcognitaMathLib {
  static imread(file) {
    let img = new Image();
    img.src = file;
    return img;
  }

  static HSV2RGB(h, s, v, a) {
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
  }
}

class WebGLMatrix {
  create() {
    return new Float32Array(16);
  }

  identity(mat) {}

  inverse(mat) {
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
      q = a * f - b * e,
      r = a * g - c * e,
      s = a * h - d * e,
      t = b * g - c * f,
      u = b * h - d * f,
      v = c * h - d * g,
      w = i * n - j * m,
      x = i * o - k * m,
      y = i * p - l * m,
      z = j * o - k * n,
      A = j * p - l * n,
      B = k * p - l * o,
      ivd = 1 / (q * B - r * A + s * z + t * y - u * x + v * w);
    return (
      (mat[0] = (f * B - g * A + h * z) * ivd),
      (mat[1] = (-b * B + c * A - d * z) * ivd),
      (mat[2] = (n * v - o * u + p * t) * ivd),
      (mat[3] = (-j * v + k * u - l * t) * ivd),
      (mat[4] = (-e * B + g * y - h * x) * ivd),
      (mat[5] = (a * B - c * y + d * x) * ivd),
      (mat[6] = (-m * v + o * s - p * r) * ivd),
      (mat[7] = (i * v - k * s + l * r) * ivd),
      (mat[8] = (e * A - f * y + h * w) * ivd),
      (mat[9] = (-a * A + b * y - d * w) * ivd),
      (mat[10] = (m * u - n * s + p * q) * ivd),
      (mat[11] = (-i * u + j * s - l * q) * ivd),
      (mat[12] = (-e * z + f * x - g * w) * ivd),
      (mat[13] = (a * z - b * x + c * w) * ivd),
      (mat[14] = (-m * t + n * r - o * q) * ivd),
      (mat[15] = (i * t - j * r + k * q) * ivd),
      mat
    );
  }
}

// WebGL Texture
class WebGLTexture {
  constructor(gl, channels, isFloat, texels, texType, texInterpolation, useMipmap) {
    this.gl = gl;
    this.type = isFloat ? gl.FLOAT : gl.UNSIGNED_BYTE;
    this.format = ""; /* ... determine format based on channels ... */
    this.glName = gl.createTexture();
    // ... initialize texture ...
  }

  bind(tex) {
    this.gl.bindTexture(this.gl.TEXTURE_2D, tex);
  }
}

// WebGL Shader
class WebGLShader {
  constructor(gl, vertexShader, fragmentShader) {
    this.gl = gl;
    this.program = "" /* ... create and link shader program ... */;
  }

  bind() {
    this.gl.useProgram(this.program);
  }
}


// Viewer
class FilterViewer {
  constructor(canvas) {
    this.canvas = canvas;
    this.gl = canvas.getContext("webgl");
  }

  loadAssets() {}

  renderScene() {}
}

// Usage
const viewer = new FilterViewer(document.getElementById("canvas_viewer"));
viewer.loadAssets();
viewer.renderScene();
