<<<<<<< HEAD
import { Shaders } from "../utils/shader.js";
import { EcognitaWeb3D as Ecognita } from "../utils/ecognita.js";

console.log(Ecognita);

=======
>>>>>>> parent of d1c1f0e (fix 路径调整)
let shaderUrl = "../assets/shader.json";

var __extends =
    (this && this.__extends) ||
    (function () {
      var extendStatics = function (d, b) {
        return (extendStatics =
          Object.setPrototypeOf ||
          ({
            __proto__: [],
          } instanceof Array &&
            function (d, b) {
              d.__proto__ = b;
            }) ||
          function (d, b) {
            for (var p in b) Object.prototype.hasOwnProperty.call(b, p) && (d[p] = b[p]);
          })(d, b);
      };
      return function (d, b) {
        function __() {
          this.constructor = d;
        }
        extendStatics(d, b), (d.prototype = null === b ? Object.create(b) : ((__.prototype = b.prototype), new __()));
      };
    })(),
  EcognitaWeb3D,
  EcognitaMathLib,
  EcognitaMathLib,
  EcognitaMathLib,
  EcognitaMathLib,
  EcognitaMathLib,
  EcognitaMathLib;
!(function (EcognitaWeb3D) {
  var Filter, RenderPipeLine;
  !(function (Filter) {
    (Filter[(Filter.LAPLACIAN = 0)] = "LAPLACIAN"),
      (Filter[(Filter.SOBEL = 1)] = "SOBEL"),
      (Filter[(Filter.GAUSSIAN = 2)] = "GAUSSIAN"),
      (Filter[(Filter.ANISTROPIC = 6)] = "ANISTROPIC"),
      (Filter[(Filter.NOISELIC = 8)] = "NOISELIC"),
      (Filter[(Filter.DoG = 9)] = "DoG"),
      (Filter[(Filter.XDoG = 10)] = "XDoG"),
      (Filter[(Filter.FXDoG = 12)] = "FXDoG");
  })((Filter = EcognitaWeb3D.Filter || (EcognitaWeb3D.Filter = {}))),
    (function (RenderPipeLine) {
      (RenderPipeLine[(RenderPipeLine.CONVOLUTION_FILTER = 0)] = "CONVOLUTION_FILTER"),
        (RenderPipeLine[(RenderPipeLine.ANISTROPIC = 1)] = "ANISTROPIC"),
        (RenderPipeLine[(RenderPipeLine.BLOOM_EFFECT = 2)] = "BLOOM_EFFECT"),
        (RenderPipeLine[(RenderPipeLine.CONVOLUTION_TWICE = 3)] = "CONVOLUTION_TWICE"),
        (RenderPipeLine[(RenderPipeLine.ABSTRACTION = 4)] = "ABSTRACTION");
    })((RenderPipeLine = EcognitaWeb3D.RenderPipeLine || (EcognitaWeb3D.RenderPipeLine = {})));
})(EcognitaWeb3D || (EcognitaWeb3D = {})),
  (function (EcognitaMathLib) {
    function imread(file) {
      var img = new Image();
      return (img.src = file), img;
    }
    EcognitaMathLib.imread = imread;
  })(EcognitaMathLib || (EcognitaMathLib = {})),
  (function (EcognitaMathLib) {
    function HSV2RGB(h, s, v, a) {
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
    EcognitaMathLib.HSV2RGB = HSV2RGB;
  })(EcognitaMathLib || (EcognitaMathLib = {})),
  EcognitaMathLib || (EcognitaMathLib = {}),
  (function (EcognitaMathLib) {
    var WebGLMatrix = (function () {
      function WebGLMatrix() {
        this.inverse = function (mat1) {
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
        };
      }
      return (
        (WebGLMatrix.prototype.create = function () {
          return new Float32Array(16);
        }),
        (WebGLMatrix.prototype.identity = function (mat) {
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
        }),
        (WebGLMatrix.prototype.multiply = function (mat1, mat2) {
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
        }),
        (WebGLMatrix.prototype.scale = function (mat1, param_scale) {
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
        }),
        (WebGLMatrix.prototype.translate = function (mat1, param_translate) {
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
              (mat[12] =
                mat1[0] * param_translate[0] + mat1[4] * param_translate[1] + mat1[8] * param_translate[2] + mat1[12]),
              (mat[13] =
                mat1[1] * param_translate[0] + mat1[5] * param_translate[1] + mat1[9] * param_translate[2] + mat1[13]),
              (mat[14] =
                mat1[2] * param_translate[0] + mat1[6] * param_translate[1] + mat1[10] * param_translate[2] + mat1[14]),
              (mat[15] =
                mat1[3] * param_translate[0] + mat1[7] * param_translate[1] + mat1[11] * param_translate[2] + mat1[15]),
              mat
            );
        }),
        (WebGLMatrix.prototype.rotate = function (mat1, angle, axis) {
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
                  ? mat1 != mat &&
                    ((mat[12] = mat1[12]), (mat[13] = mat1[13]), (mat[14] = mat1[14]), (mat[15] = mat1[15]))
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
        }),
        (WebGLMatrix.prototype.viewMatrix = function (cam, target, up) {
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
        }),
        (WebGLMatrix.prototype.perspectiveMatrix = function (fovy, aspect, near, far) {
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
        }),
        (WebGLMatrix.prototype.orthoMatrix = function (left, right, top, bottom, near, far) {
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
        }),
        (WebGLMatrix.prototype.transpose = function (mat1) {
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
        }),
        WebGLMatrix
      );
    })();
    EcognitaMathLib.WebGLMatrix = WebGLMatrix;
  })(EcognitaMathLib || (EcognitaMathLib = {})),
  (function (EcognitaMathLib) {
    var WebGLQuaternion = (function () {
      function WebGLQuaternion() {}
      return (
        (WebGLQuaternion.prototype.create = function () {
          return new Float32Array(4);
        }),
        (WebGLQuaternion.prototype.identity = function (qat) {
          return (qat[0] = 0), (qat[1] = 0), (qat[2] = 0), (qat[3] = 1), qat;
        }),
        (WebGLQuaternion.prototype.inverse = function (qat) {
          var out_qat = this.create();
          return (out_qat[0] = -qat[0]), (out_qat[1] = -qat[1]), (out_qat[2] = -qat[2]), (out_qat[3] = qat[3]), out_qat;
        }),
        (WebGLQuaternion.prototype.normalize = function (qat) {
          var x = qat[0],
            y = qat[1],
            z = qat[2],
            w = qat[3],
            l = Math.sqrt(x * x + y * y + z * z + w * w);
          return (
            0 === l
              ? ((qat[0] = 0), (qat[1] = 0), (qat[2] = 0), (qat[3] = 0))
              : ((l = 1 / l), (qat[0] = x * l), (qat[1] = y * l), (qat[2] = z * l), (qat[3] = w * l)),
            qat
          );
        }),
        (WebGLQuaternion.prototype.multiply = function (qat1, qat2) {
          var out_qat = this.create(),
            ax = qat1[0],
            ay = qat1[1],
            az = qat1[2],
            aw = qat1[3],
            bx = qat2[0],
            by = qat2[1],
            bz = qat2[2],
            bw = qat2[3];
          return (
            (out_qat[0] = ax * bw + aw * bx + ay * bz - az * by),
            (out_qat[1] = ay * bw + aw * by + az * bx - ax * bz),
            (out_qat[2] = az * bw + aw * bz + ax * by - ay * bx),
            (out_qat[3] = aw * bw - ax * bx - ay * by - az * bz),
            out_qat
          );
        }),
        (WebGLQuaternion.prototype.rotate = function (angle, axis) {
          var sq = Math.sqrt(axis[0] * axis[0] + axis[1] * axis[1] + axis[2] * axis[2]);
          if (sq) {
            var a = axis[0],
              b = axis[1],
              c = axis[2];
            1 != sq && ((a *= sq = 1 / sq), (b *= sq), (c *= sq));
            var s = Math.sin(0.5 * angle),
              out_qat = this.create();
            return (
              (out_qat[0] = a * s),
              (out_qat[1] = b * s),
              (out_qat[2] = c * s),
              (out_qat[3] = Math.cos(0.5 * angle)),
              out_qat
            );
          }
          console.log("need a axis value");
        }),
        (WebGLQuaternion.prototype.ToV3 = function (p_v3, q) {
          var out_p = new Array(3),
            inv_q = this.inverse(q),
            in_p = this.create();
          (in_p[0] = p_v3[0]), (in_p[1] = p_v3[1]), (in_p[2] = p_v3[2]);
          var p_invq = this.multiply(inv_q, in_p),
            q_p_invq = this.multiply(p_invq, q);
          return (out_p[0] = q_p_invq[0]), (out_p[1] = q_p_invq[1]), (out_p[2] = q_p_invq[2]), out_p;
        }),
        (WebGLQuaternion.prototype.ToMat4x4 = function (q) {
          var out_mat = new Float32Array(16),
            x = q[0],
            y = q[1],
            z = q[2],
            w = q[3],
            x2 = x + x,
            y2 = y + y,
            z2 = z + z,
            xx = x * x2,
            xy = x * y2,
            xz = x * z2,
            yy = y * y2,
            yz = y * z2,
            zz = z * z2,
            wx = w * x2,
            wy = w * y2,
            wz = w * z2;
          return (
            (out_mat[0] = 1 - (yy + zz)),
            (out_mat[1] = xy - wz),
            (out_mat[2] = xz + wy),
            (out_mat[3] = 0),
            (out_mat[4] = xy + wz),
            (out_mat[5] = 1 - (xx + zz)),
            (out_mat[6] = yz - wx),
            (out_mat[7] = 0),
            (out_mat[8] = xz - wy),
            (out_mat[9] = yz + wx),
            (out_mat[10] = 1 - (xx + yy)),
            (out_mat[11] = 0),
            (out_mat[12] = 0),
            (out_mat[13] = 0),
            (out_mat[14] = 0),
            (out_mat[15] = 1),
            out_mat
          );
        }),
        (WebGLQuaternion.prototype.slerp = function (qtn1, qtn2, time) {
          if (!(time < 0 || time > 1)) {
            var out_q = this.create(),
              ht = qtn1[0] * qtn2[0] + qtn1[1] * qtn2[1] + qtn1[2] * qtn2[2] + qtn1[3] * qtn2[3],
              hs = 1 - ht * ht;
            if (hs <= 0) (out_q[0] = qtn1[0]), (out_q[1] = qtn1[1]), (out_q[2] = qtn1[2]), (out_q[3] = qtn1[3]);
            else if (((hs = Math.sqrt(hs)), Math.abs(hs) < 1e-4))
              (out_q[0] = 0.5 * qtn1[0] + 0.5 * qtn2[0]),
                (out_q[1] = 0.5 * qtn1[1] + 0.5 * qtn2[1]),
                (out_q[2] = 0.5 * qtn1[2] + 0.5 * qtn2[2]),
                (out_q[3] = 0.5 * qtn1[3] + 0.5 * qtn2[3]);
            else {
              var ph = Math.acos(ht),
                pt = ph * time,
                t0 = Math.sin(ph - pt) / hs,
                t1 = Math.sin(pt) / hs;
              (out_q[0] = qtn1[0] * t0 + qtn2[0] * t1),
                (out_q[1] = qtn1[1] * t0 + qtn2[1] * t1),
                (out_q[2] = qtn1[2] * t0 + qtn2[2] * t1),
                (out_q[3] = qtn1[3] * t0 + qtn2[3] * t1);
            }
            return out_q;
          }
          console.log("parameter time's setting is wrong!");
        }),
        WebGLQuaternion
      );
    })();
    EcognitaMathLib.WebGLQuaternion = WebGLQuaternion;
  })(EcognitaMathLib || (EcognitaMathLib = {})),
  (function (EcognitaMathLib) {
    function GetGLTypeSize(type) {
      switch (type) {
        case gl.BYTE:
        case gl.UNSIGNED_BYTE:
          return 1;
        case gl.SHORT:
        case gl.UNSIGNED_SHORT:
          return 2;
        case gl.INT:
        case gl.UNSIGNED_INT:
        case gl.FLOAT:
          return 4;
        default:
          return 0;
      }
    }
    EcognitaMathLib.GetGLTypeSize = GetGLTypeSize;
    var WebGL_Texture = (function () {
      function WebGL_Texture(channels, isFloat, texels, texType, texInterpolation, useMipmap) {
        void 0 === texType && (texType = gl.REPEAT),
          void 0 === texInterpolation && (texInterpolation = gl.LINEAR),
          void 0 === useMipmap && (useMipmap = !0),
          (this.type = isFloat ? gl.FLOAT : gl.UNSIGNED_BYTE),
          (this.format = [gl.LUMINANCE, gl.RG, gl.RGB, gl.RGBA][channels - 1]),
          (this.glName = gl.createTexture()),
          this.bind(this.glName),
          gl.texImage2D(gl.TEXTURE_2D, 0, this.format, this.format, this.type, texels),
          useMipmap && gl.generateMipmap(gl.TEXTURE_2D),
          gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_MIN_FILTER, texInterpolation),
          gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_MAG_FILTER, texInterpolation),
          gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_WRAP_S, texType),
          gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_WRAP_T, texType),
          (this.texture = this.glName),
          this.bind(null);
      }
      return (
        (WebGL_Texture.prototype.bind = function (tex) {
          gl.bindTexture(gl.TEXTURE_2D, tex);
        }),
        WebGL_Texture
      );
    })();
    EcognitaMathLib.WebGL_Texture = WebGL_Texture;
    var WebGL_CubeMapTexture = (function () {
      function WebGL_CubeMapTexture(texArray) {
        (this.cubeSource = texArray),
          (this.cubeTarget = new Array(
            gl.TEXTURE_CUBE_MAP_POSITIVE_X,
            gl.TEXTURE_CUBE_MAP_POSITIVE_Y,
            gl.TEXTURE_CUBE_MAP_POSITIVE_Z,
            gl.TEXTURE_CUBE_MAP_NEGATIVE_X,
            gl.TEXTURE_CUBE_MAP_NEGATIVE_Y,
            gl.TEXTURE_CUBE_MAP_NEGATIVE_Z
          )),
          this.loadCubeTexture(),
          (this.cubeTexture = void 0);
      }
      return (
        (WebGL_CubeMapTexture.prototype.loadCubeTexture = function () {
          var _this = this,
            cubeImage = new Array(),
            loadFlagCnt = 0;
          this.cubeImage = cubeImage;
          for (var i = 0; i < this.cubeSource.length; i++)
            (cubeImage[i] = new Object()),
              (cubeImage[i].data = new Image()),
              (cubeImage[i].data.src = this.cubeSource[i]),
              (cubeImage[i].data.onload = function () {
                ++loadFlagCnt == _this.cubeSource.length && _this.generateCubeMap();
              });
        }),
        (WebGL_CubeMapTexture.prototype.generateCubeMap = function () {
          var tex = gl.createTexture();
          gl.bindTexture(gl.TEXTURE_CUBE_MAP, tex);
          for (var j = 0; j < this.cubeSource.length; j++)
            gl.texImage2D(this.cubeTarget[j], 0, gl.RGBA, gl.RGBA, gl.UNSIGNED_BYTE, this.cubeImage[j].data);
          gl.generateMipmap(gl.TEXTURE_CUBE_MAP),
            gl.texParameteri(gl.TEXTURE_CUBE_MAP, gl.TEXTURE_MIN_FILTER, gl.LINEAR),
            gl.texParameteri(gl.TEXTURE_CUBE_MAP, gl.TEXTURE_MAG_FILTER, gl.LINEAR),
            gl.texParameteri(gl.TEXTURE_CUBE_MAP, gl.TEXTURE_WRAP_S, gl.CLAMP_TO_EDGE),
            gl.texParameteri(gl.TEXTURE_CUBE_MAP, gl.TEXTURE_WRAP_T, gl.CLAMP_TO_EDGE),
            (this.cubeTexture = tex),
            gl.bindTexture(gl.TEXTURE_CUBE_MAP, null);
        }),
        WebGL_CubeMapTexture
      );
    })();
    EcognitaMathLib.WebGL_CubeMapTexture = WebGL_CubeMapTexture;
    var WebGL_RenderTarget = (function () {
      function WebGL_RenderTarget() {
        this.glName = gl.createFramebuffer();
      }
      return (
        (WebGL_RenderTarget.prototype.bind = function () {
          gl.bindFramebuffer(gl.FRAMEBUFFER, this.glName);
        }),
        (WebGL_RenderTarget.prototype.unbind = function () {
          gl.bindFramebuffer(gl.FRAMEBUFFER, null);
        }),
        (WebGL_RenderTarget.prototype.attachTexture = function (texture, index) {
          gl.framebufferTexture2D(gl.FRAMEBUFFER, gl.COLOR_ATTACHMENT0 + index, gl.TEXTURE_2D, texture.glName, 0);
        }),
        (WebGL_RenderTarget.prototype.detachTexture = function (index) {
          gl.framebufferTexture2D(gl.FRAMEBUFFER, gl.COLOR_ATTACHMENT0 + index, gl.TEXTURE_2D, null, 0);
        }),
        (WebGL_RenderTarget.prototype.drawBuffers = function (numBufs) {
          for (var buffers = [], i = 0; i < numBufs; ++i) buffers.push(gl.COLOR_ATTACHMENT0 + i);
          multiBufExt.drawBuffersWEBGL(buffers);
        }),
        WebGL_RenderTarget
      );
    })();
    EcognitaMathLib.WebGL_RenderTarget = WebGL_RenderTarget;
    var WebGL_Shader = (function () {
      function WebGL_Shader(shaderDict, vert, frag) {
        (this.vertex = this.createShaderObject(shaderDict, vert, !1)),
          (this.fragment = this.createShaderObject(shaderDict, frag, !0)),
          (this.program = gl.createProgram()),
          gl.attachShader(this.program, this.vertex),
          gl.attachShader(this.program, this.fragment),
          gl.linkProgram(this.program),
          (this.uniforms = {}),
          gl.getProgramParameter(this.program, gl.LINK_STATUS) || alert("Could not initialise shaders");
      }
      return (
        (WebGL_Shader.prototype.bind = function () {
          gl.useProgram(this.program);
        }),
        (WebGL_Shader.prototype.createShaderObject = function (shaderDict, name, isFragment) {
          var shaderSource = this.resolveShaderSource(shaderDict, name),
            shaderObject = gl.createShader(isFragment ? gl.FRAGMENT_SHADER : gl.VERTEX_SHADER);
          if (
            (gl.shaderSource(shaderObject, shaderSource),
            gl.compileShader(shaderObject),
            !gl.getShaderParameter(shaderObject, gl.COMPILE_STATUS))
          ) {
            for (var lines = shaderSource.split("\n"), i = 0; i < lines.length; ++i)
              lines[i] = ("   " + (i + 1)).slice(-4) + " | " + lines[i];
            throw (
              ((shaderSource = lines.join("\n")),
              new Error(
                (isFragment ? "Fragment" : "Vertex") +
                  " shader compilation error for shader '" +
                  name +
                  "':\n\n    " +
                  gl.getShaderInfoLog(shaderObject).split("\n").join("\n    ") +
                  "\nThe expanded shader source code was:\n\n" +
                  shaderSource
              ))
            );
          }
          return shaderObject;
        }),
        (WebGL_Shader.prototype.resolveShaderSource = function (shaderDict, name) {
          if (!(name in shaderDict)) throw new Error("Unable to find shader source for '" + name + "'");
          for (
            var shaderSource = shaderDict[name], pattern = new RegExp('#include "(.+)"'), match;
            (match = pattern.exec(shaderSource));

          )
            shaderSource =
              shaderSource.slice(0, match.index) +
              this.resolveShaderSource(shaderDict, match[1]) +
              shaderSource.slice(match.index + match[0].length);
          return shaderSource;
        }),
        (WebGL_Shader.prototype.uniformIndex = function (name) {
          return (
            name in this.uniforms || (this.uniforms[name] = gl.getUniformLocation(this.program, name)),
            this.uniforms[name]
          );
        }),
        (WebGL_Shader.prototype.uniformTexture = function (name, texture) {
          var id = this.uniformIndex(name);
          -1 != id && gl.uniform1i(id, texture.boundUnit);
        }),
        (WebGL_Shader.prototype.uniformF = function (name, f) {
          var id = this.uniformIndex(name);
          -1 != id && gl.uniform1f(id, f);
        }),
        (WebGL_Shader.prototype.uniform2F = function (name, f1, f2) {
          var id = this.uniformIndex(name);
          -1 != id && gl.uniform2f(id, f1, f2);
        }),
        WebGL_Shader
      );
    })();
    EcognitaMathLib.WebGL_Shader = WebGL_Shader;
    var WebGL_VertexBuffer = (function () {
      function WebGL_VertexBuffer() {
        (this.attributes = []), (this.elementSize = 0);
      }
      return (
        (WebGL_VertexBuffer.prototype.bind = function (shader) {
          gl.bindBuffer(gl.ARRAY_BUFFER, this.glName);
          for (var i = 0; i < this.attributes.length; ++i)
            if (
              ((this.attributes[i].index = gl.getAttribLocation(shader.program, this.attributes[i].name)),
              this.attributes[i].index >= 0)
            ) {
              var attr = this.attributes[i];
              gl.enableVertexAttribArray(attr.index),
                gl.vertexAttribPointer(attr.index, attr.size, attr.type, attr.norm, this.elementSize, attr.offset);
            }
        }),
        (WebGL_VertexBuffer.prototype.release = function () {
          for (var i = 0; i < this.attributes.length; ++i)
            this.attributes[i].index >= 0 &&
              (gl.disableVertexAttribArray(this.attributes[i].index), (this.attributes[i].index = -1));
        }),
        (WebGL_VertexBuffer.prototype.addAttribute = function (name, size, type, norm) {
          this.attributes.push({
            name: name,
            size: size,
            type: type,
            norm: norm,
            offset: this.elementSize,
            index: -1,
          }),
            (this.elementSize += size * GetGLTypeSize(type));
        }),
        (WebGL_VertexBuffer.prototype.addAttributes = function (attrArray, sizeArray) {
          for (var i = 0; i < attrArray.length; i++) this.addAttribute(attrArray[i], sizeArray[i], gl.FLOAT, !1);
        }),
        (WebGL_VertexBuffer.prototype.init = function (numVerts) {
          (this.length = numVerts),
            (this.glName = gl.createBuffer()),
            gl.bindBuffer(gl.ARRAY_BUFFER, this.glName),
            gl.bufferData(gl.ARRAY_BUFFER, this.length * this.elementSize, gl.STATIC_DRAW);
        }),
        (WebGL_VertexBuffer.prototype.copy = function (data) {
          if ((data = new Float32Array(data)).byteLength != this.length * this.elementSize)
            throw new Error("Resizing VBO during copy strongly discouraged");
          gl.bufferData(gl.ARRAY_BUFFER, data, gl.STATIC_DRAW), gl.bindBuffer(gl.ARRAY_BUFFER, null);
        }),
        (WebGL_VertexBuffer.prototype.draw = function (mode, length) {
          gl.drawArrays(mode, 0, length || this.length);
        }),
        WebGL_VertexBuffer
      );
    })();
    EcognitaMathLib.WebGL_VertexBuffer = WebGL_VertexBuffer;
    var WebGL_IndexBuffer = (function () {
      function WebGL_IndexBuffer() {}
      return (
        (WebGL_IndexBuffer.prototype.bind = function () {
          gl.bindBuffer(gl.ELEMENT_ARRAY_BUFFER, this.glName);
        }),
        (WebGL_IndexBuffer.prototype.init = function (index) {
          (this.length = index.length),
            (this.glName = gl.createBuffer()),
            gl.bindBuffer(gl.ELEMENT_ARRAY_BUFFER, this.glName),
            gl.bufferData(gl.ELEMENT_ARRAY_BUFFER, new Int16Array(index), gl.STATIC_DRAW),
            gl.bindBuffer(gl.ELEMENT_ARRAY_BUFFER, null);
        }),
        (WebGL_IndexBuffer.prototype.draw = function (mode, length) {
          gl.drawElements(mode, length || this.length, gl.UNSIGNED_SHORT, 0);
        }),
        WebGL_IndexBuffer
      );
    })();
    EcognitaMathLib.WebGL_IndexBuffer = WebGL_IndexBuffer;
    var WebGL_FrameBuffer = (function () {
      function WebGL_FrameBuffer(width, height) {
        (this.width = width), (this.height = height);
        var frameBuffer = gl.createFramebuffer();
        this.framebuffer = frameBuffer;
        var depthRenderBuffer = gl.createRenderbuffer();
        this.depthbuffer = depthRenderBuffer;
        var fTexture = gl.createTexture();
        this.targetTexture = fTexture;
      }
      return (
        (WebGL_FrameBuffer.prototype.bindFrameBuffer = function () {
          gl.bindFramebuffer(gl.FRAMEBUFFER, this.framebuffer);
        }),
        (WebGL_FrameBuffer.prototype.bindDepthBuffer = function () {
          gl.bindRenderbuffer(gl.RENDERBUFFER, this.depthbuffer),
            gl.renderbufferStorage(gl.RENDERBUFFER, gl.DEPTH_COMPONENT16, this.width, this.height),
            gl.framebufferRenderbuffer(gl.FRAMEBUFFER, gl.DEPTH_ATTACHMENT, gl.RENDERBUFFER, this.depthbuffer);
        }),
        (WebGL_FrameBuffer.prototype.renderToTexure = function () {
          gl.bindTexture(gl.TEXTURE_2D, this.targetTexture),
            gl.texImage2D(gl.TEXTURE_2D, 0, gl.RGBA, this.width, this.height, 0, gl.RGBA, gl.UNSIGNED_BYTE, null),
            gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_MAG_FILTER, gl.LINEAR),
            gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_MIN_FILTER, gl.LINEAR),
            gl.framebufferTexture2D(gl.FRAMEBUFFER, gl.COLOR_ATTACHMENT0, gl.TEXTURE_2D, this.targetTexture, 0);
        }),
        (WebGL_FrameBuffer.prototype.renderToShadowTexure = function () {
          gl.bindTexture(gl.TEXTURE_2D, this.targetTexture),
            gl.texImage2D(gl.TEXTURE_2D, 0, gl.RGBA, this.width, this.height, 0, gl.RGBA, gl.UNSIGNED_BYTE, null),
            gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_MAG_FILTER, gl.LINEAR),
            gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_MIN_FILTER, gl.LINEAR),
            gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_WRAP_S, gl.CLAMP_TO_EDGE),
            gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_WRAP_T, gl.CLAMP_TO_EDGE),
            gl.framebufferTexture2D(gl.FRAMEBUFFER, gl.COLOR_ATTACHMENT0, gl.TEXTURE_2D, this.targetTexture, 0);
        }),
        (WebGL_FrameBuffer.prototype.renderToFloatTexure = function () {
          gl.bindTexture(gl.TEXTURE_2D, this.targetTexture),
            gl.texImage2D(gl.TEXTURE_2D, 0, gl.RGBA, this.width, this.height, 0, gl.RGBA, gl.FLOAT, null),
            gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_MAG_FILTER, gl.NEAREST),
            gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_MIN_FILTER, gl.NEAREST),
            gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_WRAP_S, gl.CLAMP_TO_EDGE),
            gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_WRAP_T, gl.CLAMP_TO_EDGE),
            gl.framebufferTexture2D(gl.FRAMEBUFFER, gl.COLOR_ATTACHMENT0, gl.TEXTURE_2D, this.targetTexture, 0);
        }),
        (WebGL_FrameBuffer.prototype.renderToCubeTexture = function (cubeTarget) {
          gl.bindTexture(gl.TEXTURE_CUBE_MAP, this.targetTexture);
          for (var i = 0; i < cubeTarget.length; i++)
            gl.texImage2D(cubeTarget[i], 0, gl.RGBA, this.width, this.height, 0, gl.RGBA, gl.UNSIGNED_BYTE, null);
          gl.texParameteri(gl.TEXTURE_CUBE_MAP, gl.TEXTURE_MAG_FILTER, gl.LINEAR),
            gl.texParameteri(gl.TEXTURE_CUBE_MAP, gl.TEXTURE_MIN_FILTER, gl.LINEAR),
            gl.texParameteri(gl.TEXTURE_CUBE_MAP, gl.TEXTURE_WRAP_S, gl.CLAMP_TO_EDGE),
            gl.texParameteri(gl.TEXTURE_CUBE_MAP, gl.TEXTURE_WRAP_T, gl.CLAMP_TO_EDGE);
        }),
        (WebGL_FrameBuffer.prototype.releaseCubeTex = function () {
          gl.bindTexture(gl.TEXTURE_CUBE_MAP, null),
            gl.bindRenderbuffer(gl.RENDERBUFFER, null),
            gl.bindFramebuffer(gl.FRAMEBUFFER, null);
        }),
        (WebGL_FrameBuffer.prototype.release = function () {
          gl.bindTexture(gl.TEXTURE_2D, null),
            gl.bindRenderbuffer(gl.RENDERBUFFER, null),
            gl.bindFramebuffer(gl.FRAMEBUFFER, null);
        }),
        WebGL_FrameBuffer
      );
    })();
    EcognitaMathLib.WebGL_FrameBuffer = WebGL_FrameBuffer;
  })(EcognitaMathLib || (EcognitaMathLib = {}));

EcognitaMathLib, Utils, Utils, EcognitaWeb3D, EcognitaWeb3D;
!(function (EcognitaMathLib) {
  var BoardModel = (function () {
    function BoardModel(u_position, u_color, need_normal, need_color, need_texCoord) {
      void 0 === u_position && (u_position = void 0),
        void 0 === u_color && (u_color = void 0),
        void 0 === need_normal && (need_normal = !0),
        void 0 === need_color && (need_color = !0),
        void 0 === need_texCoord && (need_texCoord = !1),
        (this.data = new Array());
      var position = [-1, 0, -1, 1, 0, -1, -1, 0, 1, 1, 0, 1],
        normal = [0, 1, 0, 0, 1, 0, 0, 1, 0, 0, 1, 0];
      this.index = [0, 1, 2, 3, 2, 1];
      for (var texCoord = [0, 0, 1, 0, 0, 1, 1, 1], i = 0; i < 4; i++) {
        if (
          (null == u_position
            ? this.data.push(position[3 * i + 0], position[3 * i + 1], position[3 * i + 2])
            : this.data.push(u_position[3 * i + 0], u_position[3 * i + 1], u_position[3 * i + 2]),
          need_normal && this.data.push(normal[3 * i + 0], normal[3 * i + 1], normal[3 * i + 2]),
          null == u_color)
        ) {
          var color = [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1];
          need_color && this.data.push(color[4 * i + 0], color[4 * i + 1], color[4 * i + 2], color[4 * i + 3]);
        } else
          need_color && this.data.push(u_color[4 * i + 0], u_color[4 * i + 1], u_color[4 * i + 2], u_color[4 * i + 3]);
        need_texCoord && this.data.push(texCoord[2 * i + 0], texCoord[2 * i + 1]);
      }
    }
    return BoardModel;
  })();
  EcognitaMathLib.BoardModel = BoardModel;
  var TorusModel = (function () {
    function TorusModel(vcrs, hcrs, vr, hr, color, need_normal, need_texture) {
      void 0 === need_texture && (need_texture = !1),
        (this.verCrossSectionSmooth = vcrs),
        (this.horCrossSectionSmooth = hcrs),
        (this.verRadius = vr),
        (this.horRadius = hr),
        (this.data = new Array()),
        (this.index = new Array()),
        (this.normal = new Array()),
        this.preCalculate(color, need_normal, need_texture);
    }
    return (
      (TorusModel.prototype.preCalculate = function (color, need_normal, need_texture) {
        void 0 === need_texture && (need_texture = !1);
        for (var i = 0; i <= this.verCrossSectionSmooth; i++)
          for (
            var verIncrement = ((2 * Math.PI) / this.verCrossSectionSmooth) * i,
              verX = Math.cos(verIncrement),
              verY = Math.sin(verIncrement),
              ii = 0;
            ii <= this.horCrossSectionSmooth;
            ii++
          ) {
            var horIncrement = ((2 * Math.PI) / this.horCrossSectionSmooth) * ii,
              horX = (verX * this.verRadius + this.horRadius) * Math.cos(horIncrement),
              horY = verY * this.verRadius,
              horZ = (verX * this.verRadius + this.horRadius) * Math.sin(horIncrement);
            if ((this.data.push(horX, horY, horZ), need_normal)) {
              var nx = verX * Math.cos(horIncrement),
                nz = verX * Math.sin(horIncrement);
              this.normal.push(nx, verY, nz), this.data.push(nx, verY, nz);
            }
            if (null == color) {
              var rgba = EcognitaMathLib.HSV2RGB((360 / this.horCrossSectionSmooth) * ii, 1, 1, 1);
              this.data.push(rgba[0], rgba[1], rgba[2], rgba[3]);
            } else this.data.push(color[0], color[1], color[2], color[3]);
            if (need_texture) {
              var rs = (1 / this.horCrossSectionSmooth) * ii,
                rt = (1 / this.verCrossSectionSmooth) * i + 0.5;
              rt > 1 && (rt -= 1), (rt = 1 - rt), this.data.push(rs, rt);
            }
          }
        for (i = 0; i < this.verCrossSectionSmooth; i++)
          for (ii = 0; ii < this.horCrossSectionSmooth; ii++)
            (verIncrement = (this.horCrossSectionSmooth + 1) * i + ii),
              this.index.push(verIncrement, verIncrement + this.horCrossSectionSmooth + 1, verIncrement + 1),
              this.index.push(
                verIncrement + this.horCrossSectionSmooth + 1,
                verIncrement + this.horCrossSectionSmooth + 2,
                verIncrement + 1
              );
      }),
      TorusModel
    );
  })();
  EcognitaMathLib.TorusModel = TorusModel;
  var ShpereModel = (function () {
    function ShpereModel(vcrs, hcrs, rad, color, need_normal, need_texture) {
      void 0 === need_texture && (need_texture = !1),
        (this.verCrossSectionSmooth = vcrs),
        (this.horCrossSectionSmooth = hcrs),
        (this.Radius = rad),
        (this.data = new Array()),
        (this.index = new Array()),
        this.preCalculate(color, need_normal, need_texture);
    }
    return (
      (ShpereModel.prototype.preCalculate = function (color, need_normal, need_texture) {
        void 0 === need_texture && (need_texture = !1);
        for (var i = 0; i <= this.verCrossSectionSmooth; i++)
          for (
            var verIncrement = (Math.PI / this.verCrossSectionSmooth) * i,
              verX = Math.cos(verIncrement),
              verY = Math.sin(verIncrement),
              ii = 0;
            ii <= this.horCrossSectionSmooth;
            ii++
          ) {
            var horIncrement = ((2 * Math.PI) / this.horCrossSectionSmooth) * ii,
              horX = verY * this.Radius * Math.cos(horIncrement),
              horY = verX * this.Radius,
              horZ = verY * this.Radius * Math.sin(horIncrement);
            if ((this.data.push(horX, horY, horZ), need_normal)) {
              var nx = verY * Math.cos(horIncrement),
                nz = verY * Math.sin(horIncrement);
              this.data.push(nx, verX, nz);
            }
            if (null == color) {
              var rgba = EcognitaMathLib.HSV2RGB((360 / this.horCrossSectionSmooth) * i, 1, 1, 1);
              this.data.push(rgba[0], rgba[1], rgba[2], rgba[3]);
            } else this.data.push(color[0], color[1], color[2], color[3]);
            need_texture &&
              this.data.push(1 - (1 / this.horCrossSectionSmooth) * ii, (1 / this.verCrossSectionSmooth) * i);
          }
        for (i = 0; i < this.verCrossSectionSmooth; i++)
          for (ii = 0; ii < this.horCrossSectionSmooth; ii++)
            (verIncrement = (this.horCrossSectionSmooth + 1) * i + ii),
              this.index.push(verIncrement, verIncrement + 1, verIncrement + this.horCrossSectionSmooth + 2),
              this.index.push(
                verIncrement,
                verIncrement + this.horCrossSectionSmooth + 2,
                verIncrement + this.horCrossSectionSmooth + 1
              );
      }),
      ShpereModel
    );
  })();
  EcognitaMathLib.ShpereModel = ShpereModel;
  var CubeModel = (function () {
    function CubeModel(side, color, need_normal, need_texture) {
      void 0 === need_texture && (need_texture = !1),
        (this.side = side),
        (this.data = new Array()),
        (this.index = [
          0, 1, 2, 0, 2, 3, 4, 5, 6, 4, 6, 7, 8, 9, 10, 8, 10, 11, 12, 13, 14, 12, 14, 15, 16, 17, 18, 16, 18, 19, 20,
          21, 22, 20, 22, 23,
        ]);
      for (
        var hs = 0.5 * side,
          pos = [
            -hs,
            -hs,
            hs,
            hs,
            -hs,
            hs,
            hs,
            hs,
            hs,
            -hs,
            hs,
            hs,
            -hs,
            -hs,
            -hs,
            -hs,
            hs,
            -hs,
            hs,
            hs,
            -hs,
            hs,
            -hs,
            -hs,
            -hs,
            hs,
            -hs,
            -hs,
            hs,
            hs,
            hs,
            hs,
            hs,
            hs,
            hs,
            -hs,
            -hs,
            -hs,
            -hs,
            hs,
            -hs,
            -hs,
            hs,
            -hs,
            hs,
            -hs,
            -hs,
            hs,
            hs,
            -hs,
            -hs,
            hs,
            hs,
            -hs,
            hs,
            hs,
            hs,
            hs,
            -hs,
            hs,
            -hs,
            -hs,
            -hs,
            -hs,
            -hs,
            hs,
            -hs,
            hs,
            hs,
            -hs,
            hs,
            -hs,
          ],
          normal = [
            -1, -1, 1, 1, -1, 1, 1, 1, 1, -1, 1, 1, -1, -1, -1, -1, 1, -1, 1, 1, -1, 1, -1, -1, -1, 1, -1, -1, 1, 1, 1,
            1, 1, 1, 1, -1, -1, -1, -1, 1, -1, -1, 1, -1, 1, -1, -1, 1, 1, -1, -1, 1, 1, -1, 1, 1, 1, 1, -1, 1, -1, -1,
            -1, -1, -1, 1, -1, 1, 1, -1, 1, -1,
          ],
          col = new Array(),
          i = 0;
        i < pos.length / 3;
        i++
      ) {
        if (null != color) var tc = color;
        else tc = EcognitaMathLib.HSV2RGB((360 / pos.length / 3) * i, 1, 1, 1);
        col.push(tc[0], tc[1], tc[2], tc[3]);
      }
      for (
        var st = [
            0, 0, 1, 0, 1, 1, 0, 1, 0, 0, 1, 0, 1, 1, 0, 1, 0, 0, 1, 0, 1, 1, 0, 1, 0, 0, 1, 0, 1, 1, 0, 1, 0, 0, 1, 0,
            1, 1, 0, 1, 0, 0, 1, 0, 1, 1, 0, 1,
          ],
          cubeVertexNum = 24,
          i = 0;
        i < 24;
        i++
      )
        this.data.push(pos[3 * i + 0], pos[3 * i + 1], pos[3 * i + 2]),
          need_normal && this.data.push(normal[3 * i + 0], normal[3 * i + 1], normal[3 * i + 2]),
          this.data.push(col[4 * i + 0], col[4 * i + 1], col[4 * i + 2], col[4 * i + 3]),
          need_texture && this.data.push(st[2 * i + 0], st[2 * i + 1]);
    }
    return CubeModel;
  })();
  EcognitaMathLib.CubeModel = CubeModel;
})(EcognitaMathLib || (EcognitaMathLib = {})),
  (function (Utils) {
    var HashSet = (function () {
      function HashSet() {
        this.items = {};
      }
      return (
        (HashSet.prototype.set = function (key, value) {
          this.items[key] = value;
        }),
        (HashSet.prototype.delete = function (key) {
          return delete this.items[key];
        }),
        (HashSet.prototype.has = function (key) {
          return key in this.items;
        }),
        (HashSet.prototype.get = function (key) {
          return this.items[key];
        }),
        (HashSet.prototype.len = function () {
          return Object.keys(this.items).length;
        }),
        (HashSet.prototype.forEach = function (f) {
          for (var k in this.items) f(k, this.items[k]);
        }),
        HashSet
      );
    })();
    Utils.HashSet = HashSet;
  })(Utils || (Utils = {})),
  (function (Utils) {
    var FilterViewerUI = (function () {
      function FilterViewerUI(data) {
        var _this = this;
        (this.gui = new dat.gui.GUI()),
          (this.data = data),
          this.gui.remember(data),
          (this.uiController = new Utils.HashSet()),
          (this.folderHashSet = new Utils.HashSet()),
          this.folderHashSet.set("f", "Filter"),
          (this.folderName = []),
          this.folderHashSet.forEach(function (k, v) {
            _this.folderName.push(k);
          }),
          this.initData(),
          this.initFolder();
      }
      return (
        (FilterViewerUI.prototype.initFolder = function () {
          var _this = this;
          this.folderName.forEach(function (fn) {
            var f = _this.gui.addFolder(_this.folderHashSet.get(fn));
            for (var key in _this.data) {
              var f_name = key.split("_");
              if (key.includes("_") && f_name[0] == fn) {
                var c = f.add(_this.data, key).listen();
                _this.uiController.set(key, c);
              }
            }
          });
        }),
        (FilterViewerUI.prototype.initData = function () {
          for (var key in this.data) key.includes("_") || this.gui.add(this.data, key);
        }),
        FilterViewerUI
      );
    })();
    Utils.FilterViewerUI = FilterViewerUI;
  })(Utils || (Utils = {})),
  (function (EcognitaWeb3D) {
    var WebGLEnv = (function () {
      function WebGLEnv(cvs) {
        this.chkWebGLEnv(cvs);
      }
      return (
        (WebGLEnv.prototype.loadTexture = function (file_name, isFloat, glType, glInterType, useMipmap, channel) {
          var _this = this;
          void 0 === isFloat && (isFloat = !1),
            void 0 === glType && (glType = gl.CLAMP_TO_EDGE),
            void 0 === glInterType && (glInterType = gl.LINEAR),
            void 0 === useMipmap && (useMipmap = !0),
            void 0 === channel && (channel = 4);
          var tex = null,
            image = EcognitaMathLib.imread(file_name);
          image.onload = function () {
            (_this.canvas.width = image.width),
              (_this.canvas.height = image.height),
              _this.canvas.height > 1024 &&
                ((_this.canvas.width = _this.canvas.width * (1024 / _this.canvas.height)),
                (_this.canvas.height = 1024)),
              _this.canvas.width > 1024 &&
                ((_this.canvas.height = _this.canvas.height * (1024 / _this.canvas.width)),
                (_this.canvas.width = 1024)),
              (tex = new EcognitaMathLib.WebGL_Texture(channel, isFloat, image, glType, glInterType, useMipmap)),
              _this.Texture.set(file_name, tex);
          };
        }),
        (WebGLEnv.prototype.chkWebGLEnv = function (cvs) {
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
        }),
        (WebGLEnv.prototype.initGlobalVariables = function () {
          (this.vbo = new Array()),
            (this.ibo = new Array()),
            (this.Texture = new Utils.HashSet()),
            (this.matUtil = new EcognitaMathLib.WebGLMatrix()),
            (this.quatUtil = new EcognitaMathLib.WebGLQuaternion());
        }),
        (WebGLEnv.prototype.initGlobalMatrix = function () {
          this.MATRIX = new Utils.HashSet();
          var m = this.matUtil;
          this.MATRIX.set("mMatrix", m.identity(m.create())),
            this.MATRIX.set("vMatrix", m.identity(m.create())),
            this.MATRIX.set("pMatrix", m.identity(m.create())),
            this.MATRIX.set("vpMatrix", m.identity(m.create())),
            this.MATRIX.set("mvpMatrix", m.identity(m.create())),
            this.MATRIX.set("invMatrix", m.identity(m.create()));
        }),
        (WebGLEnv.prototype.loadExtraLibrary = function (ui_data) {
          this.ui_data = ui_data;
        }),
        (WebGLEnv.prototype.loadInternalLibrary = function (shaderlist) {
          var _this = this;
          (this.framebuffers = new Utils.HashSet()),
            (this.shaders = new Utils.HashSet()),
            (this.uniLocations = new Utils.HashSet()),
            shaderlist.forEach(function (s) {
              var shader = new EcognitaMathLib.WebGL_Shader(Shaders, s.name + "-vert", s.name + "-frag");
              _this.shaders.set(s.name, shader), _this.uniLocations.set(s.name, new Array());
            });
        }),
        (WebGLEnv.prototype.settingFrameBuffer = function (frameBufferName) {
          var fBufferWidth = this.canvas.width,
            fBufferHeight = this.canvas.height,
            frameBuffer = new EcognitaMathLib.WebGL_FrameBuffer(fBufferWidth, fBufferHeight);
          frameBuffer.bindFrameBuffer(),
            frameBuffer.bindDepthBuffer(),
            frameBuffer.renderToFloatTexure(),
            frameBuffer.release(),
            this.framebuffers.set(frameBufferName, frameBuffer);
        }),
        (WebGLEnv.prototype.renderSceneByFrameBuffer = function (framebuffer, func, texid) {
          void 0 === texid && (texid = gl.TEXTURE0),
            framebuffer.bindFrameBuffer(),
            func(),
            gl.activeTexture(texid),
            gl.bindTexture(gl.TEXTURE_2D, framebuffer.targetTexture);
        }),
        (WebGLEnv.prototype.renderBoardByFrameBuffer = function (shader, vbo, ibo, func, use_fb, texid, fb) {
          void 0 === use_fb && (use_fb = !1),
            void 0 === texid && (texid = gl.TEXTURE0),
            void 0 === fb && (fb = void 0),
            shader.bind(),
            use_fb ? fb.bindFrameBuffer() : gl.bindFramebuffer(gl.FRAMEBUFFER, null),
            gl.clearColor(0, 0, 0, 1),
            gl.clearDepth(1),
            gl.clear(gl.COLOR_BUFFER_BIT | gl.DEPTH_BUFFER_BIT),
            vbo.bind(shader),
            ibo.bind(),
            func(),
            ibo.draw(gl.TRIANGLES),
            use_fb && (gl.activeTexture(texid), gl.bindTexture(gl.TEXTURE_2D, fb.targetTexture));
        }),
        WebGLEnv
      );
    })();
    EcognitaWeb3D.WebGLEnv = WebGLEnv;
  })(EcognitaWeb3D || (EcognitaWeb3D = {})),
  (function (EcognitaWeb3D) {
    var FilterViewer = (function (_super) {
      function FilterViewer(cvs) {
        var _this = _super.call(this, cvs) || this;
        return (
          (_this.alpha = 50),
          (_this.beta = 50),
          (_this.gamma = 30),
          (_this.light = 100),
          (_this.clean = 0),
          (_this.pixels = 1e3),
          (_this.isoutline = !0),
          _this.imgbase64,
          _this
        );
      }
      return (
        __extends(FilterViewer, _super),
        (FilterViewer.prototype.loadAssets = function () {
          this.loadTexture("./image/k0.png", !1),
            this.loadTexture("./image/visual_rgb.png"),
            this.loadTexture("./image/lion.png", !1),
            this.loadTexture("./image/cat.jpg", !1),
            this.loadTexture("./image/noise.png", !1);
        }),
        (FilterViewer.prototype.getReqQuery = function () {
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
        }),
        (FilterViewer.prototype.regisButton = function (btn_data) {
          var _this = this;
          const betaSlider = document.getElementById("beta");
          betaSlider.addEventListener("change", function (event) {
            (_this.beta = event.target.value), _this.regisAnimeFunc(), _this.drawVector();
          });
          const gammaSlider = $("#gamma");
          gammaSlider.on("change", function (event) {
            (_this.gamma = event.target.value), _this.drawVector();
          });
          const outlineCB = $("#outline");
          outlineCB.on("change", function (event) {
            (_this.isoutline = outlineCB.is(":checked")), _this.regisAnimeFunc(), _this.drawVector();
          });
          const lightSlider = $("#light");
          lightSlider.on("change", function (event) {
            (_this.light = event.target.value), _this.regisAnimeFunc(), _this.drawVector();
          });
          const cleanSlider = $("#clean");
          cleanSlider.on("change", function (event) {
            (_this.clean = event.target.value), _this.regisAnimeFunc(), _this.drawVector();
          }),
            (this.btnStatusList = new Utils.HashSet()),
            btn_data.forEach(function (btn) {
              _this.btnStatusList.set(btn.name, _this.ui_data[btn.name]);
            }),
            $("#pixels").on("change", (event) => {
              (_this.pixels = event.target.value),
                (_this.pixels = _this.pixels > 6e3 ? 6e3 : _this.pixels),
                (_this.pixels = _this.pixels < 1e3 ? 1e3 : _this.pixels),
                (event.target.value = _this.pixels);
            });
          const svgelem = $("#svgoutput");
          $("#downloadsvg").on("click", function () {
            var link = document.createElement("a");
            (link.href = svgelem.attr("src")),
              (link.download = "vectordesign.svg"),
              document.body.appendChild(link),
              link.click(),
              document.body.removeChild(link);
          }),
            $("#downloadpng").on("click", function () {
              const svgBase64 = svgelem.attr("src"),
                drawCanvas = document.createElement("canvas");
              document.body.appendChild(drawCanvas);
              const drawCtx = drawCanvas.getContext("2d");
              drawCanvas.width = _this.pixels;
              const img = new Image();
              (img.onload = () => {
                const aspect = img.width / img.height;
                (drawCanvas.height = _this.pixels / aspect),
                  drawCtx.drawImage(img, 0, 0, drawCanvas.width, drawCanvas.height);
                const link = document.createElement("a");
                (link.href = drawCanvas.toDataURL()),
                  (link.download = "vectordesign.png"),
                  document.body.appendChild(link),
                  link.click(),
                  document.body.removeChild(link),
                  document.body.removeChild(drawCanvas);
              }),
                (img.src = svgBase64);
            }),
            $("#downloadjpg").on("click", function () {
              const svgBase64 = svgelem.attr("src"),
                drawCanvas = document.createElement("canvas");
              document.body.appendChild(drawCanvas);
              const drawCtx = drawCanvas.getContext("2d");
              drawCanvas.width = _this.pixels;
              const img = new Image();
              (img.onload = () => {
                const aspect = img.width / img.height;
                (drawCanvas.height = _this.pixels / aspect),
                  (drawCtx.fillStyle = "#ffffff"),
                  drawCtx.fillRect(0, 0, drawCanvas.width, drawCanvas.height),
                  drawCtx.drawImage(img, 0, 0, drawCanvas.width, drawCanvas.height);
                const link = document.createElement("a");
                (link.href = drawCanvas.toDataURL("image/jpeg", 1)),
                  (link.download = "vectordesign.jpg"),
                  document.body.appendChild(link),
                  link.click(),
                  document.body.removeChild(link),
                  document.body.removeChild(drawCanvas);
              }),
                (img.src = svgBase64);
            }),
            $("#downloadpdf").on("click", function () {
              const svgBase64 = svgelem.attr("src"),
                drawCanvas = document.createElement("canvas");
              document.body.appendChild(drawCanvas);
              const drawCtx = drawCanvas.getContext("2d");
              drawCanvas.width = _this.pixels;
              const img = new Image();
              (img.onload = () => {
                const aspect = img.width / img.height;
                (drawCanvas.height = _this.pixels / aspect),
                  drawCtx.drawImage(img, 0, 0, drawCanvas.width, drawCanvas.height);
                const doc = new jsPDF("p", "mm", [297, 210]);
                doc.addImage(drawCanvas.toDataURL(), "PNG", 0, 0, 210, 210 / aspect),
                  doc.save("vectordesign.pdf"),
                  document.body.removeChild(drawCanvas);
              }),
                (img.src = svgBase64);
            });
        }),
        (FilterViewer.prototype.drawVector = function () {
          var _this = this;
          const url = document.getElementById("imginput").getAttribute("src"),
            img = new Image(),
            cnvs = document.getElementById("canvas1"),
            cnvsctx = cnvs.getContext("2d"),
            scnvs = document.getElementById("canvas2"),
            scnvsctx = scnvs.getContext("2d"),
            fcnvctx = document.getElementById("canvas_viewer").getContext("2d"),
            isoutline = _this.isoutline;
          (img.onload = () => {
            (scnvs.width = cnvs.width = 1024),
              (scnvs.height = cnvs.height = (1024 * img.height) / img.width),
              (cnvsctx.shadowColor = isoutline ? "#00000088" : "#00000000"),
              (cnvsctx.shadowBlur = 0),
              (cnvsctx.fillStyle = "#ffffff"),
              cnvsctx.fillRect(0, 0, cnvs.width, cnvs.height),
              (cnvsctx.filter = "brightness(" + _this.light + "%) blur(" + (100 - _this.beta) / 20 + "px)");
            for (var x = -1; x <= 1; x++)
              for (var y = -1; y <= 1; y++)
                (cnvsctx.shadowOffsetX = x),
                  (cnvsctx.shadowOffsetY = y),
                  cnvsctx.drawImage(img, 0, 0, cnvs.width, cnvs.height);
            (scnvsctx.fillStyle = "#ffffff"),
              scnvsctx.fillRect(0, 0, cnvs.width, cnvs.height),
              scnvsctx.drawImage(cnvs, 0, 0, cnvs.width, cnvs.height),
              (_this.imgbase64 = scnvs.toDataURL()),
              _this.loadTexture(_this.imgbase64, !1, gl.CLAMP_TO_EDGE, gl.LINEAR, !1),
              (_this.usrSelected = _this.imgbase64),
              setTimeout(function () {
                _this.regisAnimeFunc();
              }, 10);
          }),
            (img.src = url);
        }),
        (FilterViewer.prototype.regisUniforms = function (shader_data) {
          var _this = this;
          shader_data.forEach(function (shader) {
            var uniform_array = new Array(),
              shaderUniforms;
            shader.uniforms.forEach(function (uniform) {
              uniform_array.push(uniform);
            }),
              _this.settingUniform(shader.name, uniform_array);
          });
        }),
        (FilterViewer.prototype.regisUserParam = function (user_config) {
          (this.filterMvpMatrix = this.matUtil.identity(this.matUtil.create())),
            (this.usrSelected = user_config.user_selected);
          var default_btn_name = user_config.default_btn,
            params = this.getReqQuery();
          null == params.p || null == params.f || null == params.s || null == params.b
            ? ((this.usrPipeLine = EcognitaWeb3D.RenderPipeLine[user_config.default_pipline]),
              (this.usrFilter = EcognitaWeb3D.Filter[user_config.default_filter]),
              (this.filterShader = this.shaders.get(user_config.default_shader)))
            : ((this.usrPipeLine = EcognitaWeb3D.RenderPipeLine[params.p]),
              (this.usrFilter = EcognitaWeb3D.Filter[params.f]),
              (this.filterShader = this.shaders.get(params.s)),
              (default_btn_name = params.b)),
            (this.uiData[default_btn_name] = !0);
        }),
        (FilterViewer.prototype.initialize = function (all_data) {
          (this.uiData = all_data.ui_data),
            this.initGlobalVariables(),
            this.loadInternalLibrary(all_data.shader_data.shaderList),
            this.regisUniforms(all_data.shader_data.shaderList),
            this.regisUserParam(all_data.user_config),
            this.loadExtraLibrary(all_data.ui_data),
            this.initGlobalMatrix(),
            this.regisButton(all_data.button_data.buttonList),
            this.initModel(),
            this.regisEvent(),
            this.settingRenderPipeline();
        }),
        (FilterViewer.prototype.initModel = function () {
          var torusData = new EcognitaMathLib.TorusModel(64, 64, 1, 2, [1, 1, 1, 1], !0, !1),
            vbo_torus = new EcognitaMathLib.WebGL_VertexBuffer(),
            ibo_torus = new EcognitaMathLib.WebGL_IndexBuffer();
          this.vbo.push(vbo_torus),
            this.ibo.push(ibo_torus),
            vbo_torus.addAttribute("position", 3, gl.FLOAT, !1),
            vbo_torus.addAttribute("normal", 3, gl.FLOAT, !1),
            vbo_torus.addAttribute("color", 4, gl.FLOAT, !1),
            vbo_torus.init(torusData.data.length / 10),
            vbo_torus.copy(torusData.data),
            ibo_torus.init(torusData.index);
          var position = [-1, 1, 0, 1, 1, 0, -1, -1, 0, 1, -1, 0],
            boardData = new EcognitaMathLib.BoardModel(position, void 0, !1, !1, !0),
            vbo_board = new EcognitaMathLib.WebGL_VertexBuffer(),
            ibo_board = new EcognitaMathLib.WebGL_IndexBuffer();
          this.vbo.push(vbo_board),
            this.ibo.push(ibo_board),
            vbo_board.addAttribute("position", 3, gl.FLOAT, !1),
            vbo_board.addAttribute("texCoord", 2, gl.FLOAT, !1),
            (boardData.index = [0, 2, 1, 2, 3, 1]),
            vbo_board.init(boardData.data.length / 5),
            vbo_board.copy(boardData.data),
            ibo_board.init(boardData.index);
        }),
        (FilterViewer.prototype.renderGaussianFilter = function (horizontal, b_gaussian, tex_num) {
          void 0 === tex_num && (tex_num = 0);
          var GaussianFilterUniformLoc = this.uniLocations.get("gaussianFilter");
          gl.uniformMatrix4fv(GaussianFilterUniformLoc[0], !1, this.filterMvpMatrix),
            gl.uniform1i(GaussianFilterUniformLoc[1], tex_num),
            gl.uniform1fv(GaussianFilterUniformLoc[2], this.usrParams.gaussianWeight),
            gl.uniform1i(GaussianFilterUniformLoc[3], horizontal),
            gl.uniform1f(GaussianFilterUniformLoc[4], this.canvas.height),
            gl.uniform1f(GaussianFilterUniformLoc[5], this.canvas.width),
            gl.uniform1i(GaussianFilterUniformLoc[6], b_gaussian);
        }),
        (FilterViewer.prototype.renderFilter = function () {
          if (this.usrFilter == EcognitaWeb3D.Filter.FXDoG) {
            var FXDoGUniformLoc = this.uniLocations.get("FXDoG");
            gl.uniformMatrix4fv(FXDoGUniformLoc[0], !1, this.filterMvpMatrix),
              gl.uniform1i(FXDoGUniformLoc[1], 0),
              gl.uniform1i(FXDoGUniformLoc[2], 1),
              gl.uniform1f(FXDoGUniformLoc[3], 4.4),
              gl.uniform1f(FXDoGUniformLoc[4], 0.017),
              gl.uniform1f(FXDoGUniformLoc[5], 80),
              gl.uniform1f(FXDoGUniformLoc[6], this.canvas.height),
              gl.uniform1f(FXDoGUniformLoc[7], this.canvas.width),
              gl.uniform1i(FXDoGUniformLoc[8], this.btnStatusList.get("f_FXDoG")),
              setTimeout(() => {
                const dataURL = this.canvas.toDataURL();
                Potrace.setParameter({
                  turdsize: this.clean,
                }),
                  Potrace.loadImage(dataURL, function () {
                    console.log("FXDog LOADED");
                    const outputImg = document.getElementById("svgoutput"),
                      svgdata = "data:image/svg+xml;base64," + window.btoa(Potrace.getSVG(1));
                    outputImg.setAttribute("src", svgdata),
                      $(".nav-tabs a").removeClass("active"),
                      $(".nav-tabs a").removeClass("show"),
                      $(".tab-pane").removeClass("active"),
                      $('.nav-tabs a[href="#tab2"').addClass("active"),
                      $('.nav-tabs a[href="#tab2"').addClass("show"),
                      $("#tab2").addClass("active");
                  });
              }, 0);
          }
        }),
        (FilterViewer.prototype.settingUniform = function (shaderName, uniformIndexArray) {
          var uniLocArray = this.uniLocations.get(shaderName),
            shader = this.shaders.get(shaderName);
          uniformIndexArray.forEach(function (uniName) {
            uniLocArray.push(shader.uniformIndex(uniName));
          });
        }),
        (FilterViewer.prototype.settingRenderPipeline = function () {
          gl.enable(gl.DEPTH_TEST), gl.depthFunc(gl.LEQUAL), gl.enable(gl.CULL_FACE);
        }),
        (FilterViewer.prototype.usrSelectChange = function (btnName, val, pipeline, filter, filter_name) {
          if ((this.btnStatusList.set(btnName, val), val))
            for (var key in ((this.usrPipeLine = pipeline),
            (this.usrFilter = filter),
            (this.filterShader = this.shaders.get(filter_name)),
            this.ui_data)) {
              var f_name;
              if (key.includes("_"))
                "f" == key.split("_")[0] &&
                  key != btnName &&
                  (this.btnStatusList.set(key, !val), (this.ui_data[key] = !val));
            }
        }),
        (FilterViewer.prototype.regisEvent = function () {
          var _this = this;
          document.addEventListener("paste", (event) => {
            const fileData = event.clipboardData.files[0];
            if (!fileData.type.match("image.*")) return void alert("you should paste a image!");
            $('.nav-tabs a[href="#tab1"').trigger("click");
            const url = URL.createObjectURL(fileData);
            document.getElementById("imginput").setAttribute("src", url);
            const img = new Image(),
              cnvs = document.getElementById("canvas1"),
              cnvsctx = cnvs.getContext("2d"),
              scnvs = document.getElementById("canvas2"),
              scnvsctx = scnvs.getContext("2d");
            let imgbase64;
            (img.onload = () => {
              (scnvs.width = cnvs.width = 1024),
                (scnvs.height = cnvs.height = (1024 * img.height) / img.width),
                (cnvsctx.shadowColor = _this.isoutline ? "#00000088" : "#00000000"),
                (cnvsctx.shadowBlur = 0);
              for (var x = -1; x <= 1; x++)
                for (var y = -1; y <= 1; y++)
                  (cnvsctx.shadowOffsetX = x),
                    (cnvsctx.shadowOffsetY = y),
                    cnvsctx.drawImage(img, 0, 0, cnvs.width, cnvs.height);
              (scnvsctx.fillStyle = "#ffffff"),
                scnvsctx.fillRect(0, 0, cnvs.width, cnvs.height),
                scnvsctx.drawImage(cnvs, 0, 0, cnvs.width, cnvs.height),
                (imgbase64 = scnvs.toDataURL()),
                _this.loadTexture(imgbase64, !1, gl.CLAMP_TO_EDGE, gl.LINEAR, !1),
                (_this.usrSelected = imgbase64),
                setTimeout(function () {
                  _this.regisAnimeFunc();
                }, 100);
            }),
              (img.src = url);
          });
          const fileInput = document.getElementById("myfile");
          fileInput.onchange = () => {
            var fileData = fileInput.files[0];
            if (!fileData.type.match("image.*")) return void alert("you should upload a image!");
            $('.nav-tabs a[href="#tab1"').trigger("click");
            const url = URL.createObjectURL(fileData);
            document.getElementById("imginput").setAttribute("src", url);
            const img = new Image(),
              cnvs = document.getElementById("canvas1"),
              cnvsctx = cnvs.getContext("2d"),
              scnvs = document.getElementById("canvas2"),
              scnvsctx = scnvs.getContext("2d");
            let imgbase64;
            (img.onload = () => {
              (scnvs.width = cnvs.width = 1024),
                (scnvs.height = cnvs.height = (1024 * img.height) / img.width),
                (cnvsctx.shadowColor = _this.isoutline ? "#00000088" : "#00000000"),
                (cnvsctx.shadowBlur = 0);
              for (var x = -1; x <= 1; x++)
                for (var y = -1; y <= 1; y++)
                  (cnvsctx.shadowOffsetX = x),
                    (cnvsctx.shadowOffsetY = y),
                    cnvsctx.drawImage(img, 0, 0, cnvs.width, cnvs.height);
              (scnvsctx.fillStyle = "#ffffff"),
                scnvsctx.fillRect(0, 0, cnvs.width, cnvs.height),
                scnvsctx.drawImage(cnvs, 0, 0, cnvs.width, cnvs.height),
                (imgbase64 = scnvs.toDataURL()),
                _this.loadTexture(imgbase64, !1, gl.CLAMP_TO_EDGE, gl.LINEAR, !1),
                (_this.usrSelected = imgbase64),
                setTimeout(function () {
                  _this.regisAnimeFunc();
                }, 100);
            }),
              (img.src = url);
          };
        }),
        (FilterViewer.prototype.regisFrameBuffer = function (num) {
          for (var fArray = Array(num), i = 0; i < num; i++) {
            this.settingFrameBuffer("frameBuffer" + i);
            var fb = this.framebuffers.get("frameBuffer" + i);
            fArray[i] = fb;
          }
          return fArray;
        }),
        (FilterViewer.prototype.regisAnimeFunc = function () {
          var _this = this,
            cnt = 0,
            cnt1 = 0,
            lightDirection = [-0.577, 0.577, 0.577],
            m = this.matUtil,
            q = this.quatUtil;
          this.usrQuaternion = q.identity(q.create());
          var sceneShader = this.shaders.get("filterScene"),
            sceneUniformLoc = this.uniLocations.get("filterScene"),
            vbo_torus = this.vbo[0],
            ibo_torus = this.ibo[0],
            vbo_board = this.vbo[1],
            ibo_board = this.ibo[1],
            mMatrix = this.MATRIX.get("mMatrix"),
            vMatrix = this.MATRIX.get("vMatrix"),
            pMatrix = this.MATRIX.get("pMatrix"),
            vpMatrix = this.MATRIX.get("vpMatrix"),
            mvpMatrix = this.MATRIX.get("mvpMatrix"),
            invMatrix = this.MATRIX.get("invMatrix"),
            specCptShader = this.shaders.get("specCpt"),
            uniLocation_spec = this.uniLocations.get("specCpt"),
            synthShader = this.shaders.get("synth"),
            uniLocation_synth = this.uniLocations.get("synth"),
            luminanceShader = this.shaders.get("luminance"),
            uniLocation_luminance = this.uniLocations.get("luminance"),
            TFShader = this.shaders.get("TF"),
            uniLocation_TF = this.uniLocations.get("TF"),
            ETFShader = this.shaders.get("ETF"),
            uniLocation_ETF = this.uniLocations.get("ETF"),
            PFDoGShader = this.shaders.get("P_FDoG"),
            uniLocation_PFDoG = this.uniLocations.get("P_FDoG"),
            PFXDoGShader = this.shaders.get("P_FXDoG"),
            uniLocation_PFXDoG = this.uniLocations.get("P_FXDoG"),
            FXDoGShader = this.shaders.get("FXDoG"),
            uniLocation_FXDoG = this.uniLocations.get("FXDoG"),
            SSTShader = this.shaders.get("SST"),
            uniLocation_SST = this.uniLocations.get("SST"),
            GAUShader = this.shaders.get("Gaussian_K"),
            uniLocation_GAU = this.uniLocations.get("Gaussian_K"),
            TFMShader = this.shaders.get("TFM"),
            uniLocation_TFM = this.uniLocations.get("TFM"),
            AKFShader = this.shaders.get("AKF"),
            AKFUniformLoc = this.uniLocations.get("AKF"),
            fb = this.regisFrameBuffer(5),
            loop;
          (function () {
            ++cnt % 2 == 0 && cnt1++;
            var rad = ((cnt % 360) * Math.PI) / 180,
              eyePosition = new Array(),
              camUpDirection = new Array();
            (eyePosition = q.ToV3([0, 20, 0], _this.usrQuaternion)),
              (camUpDirection = q.ToV3([0, 0, -1], _this.usrQuaternion)),
              (vMatrix = m.viewMatrix(eyePosition, [0, 0, 0], camUpDirection)),
              (pMatrix = m.perspectiveMatrix(90, _this.canvas.width / _this.canvas.height, 0.1, 100)),
              (vpMatrix = m.multiply(pMatrix, vMatrix)),
              (vMatrix = m.viewMatrix([0, 0, 0.5], [0, 0, 0], [0, 1, 0])),
              (pMatrix = m.orthoMatrix(-1, 1, 1, -1, 0.1, 1)),
              (_this.filterMvpMatrix = m.multiply(pMatrix, vMatrix));
            var inTex = _this.Texture.get(_this.usrSelected);
            if (_this.usrPipeLine == EcognitaWeb3D.RenderPipeLine.CONVOLUTION_FILTER)
              null != inTex
                ? (gl.activeTexture(gl.TEXTURE0), inTex.bind(inTex.texture))
                : _this.renderSceneByFrameBuffer(fb[0], RenderSimpleScene),
                _this.renderBoardByFrameBuffer(_this.filterShader, vbo_board, ibo_board, function () {
                  _this.renderFilter();
                });
            else if (_this.usrPipeLine == EcognitaWeb3D.RenderPipeLine.BLOOM_EFFECT) {
              null != inTex
                ? (gl.activeTexture(gl.TEXTURE0), inTex.bind(inTex.texture))
                : _this.renderSceneByFrameBuffer(fb[0], RenderSimpleScene),
                _this.renderBoardByFrameBuffer(
                  luminanceShader,
                  vbo_board,
                  ibo_board,
                  function () {
                    gl.uniformMatrix4fv(uniLocation_luminance[0], !1, _this.filterMvpMatrix),
                      gl.uniform1i(uniLocation_luminance[1], 0),
                      gl.uniform1f(uniLocation_luminance[2], 0.5);
                  },
                  !0,
                  gl.TEXTURE1,
                  fb[1]
                );
              for (var sample_count = 9, _i = 0; _i < 9; _i++)
                _this.renderBoardByFrameBuffer(
                  _this.filterShader,
                  vbo_board,
                  ibo_board,
                  function () {
                    _this.renderGaussianFilter(!0, _this.btnStatusList.get("f_BloomEffect"), 1);
                  },
                  !0,
                  gl.TEXTURE0,
                  fb[0]
                ),
                  _this.renderBoardByFrameBuffer(
                    _this.filterShader,
                    vbo_board,
                    ibo_board,
                    function () {
                      _this.renderGaussianFilter(!1, _this.btnStatusList.get("f_BloomEffect"));
                    },
                    !0,
                    gl.TEXTURE1,
                    fb[1]
                  );
              null != inTex
                ? (gl.activeTexture(gl.TEXTURE0), inTex.bind(inTex.texture))
                : _this.renderBoardByFrameBuffer(
                    _this.filterShader,
                    vbo_board,
                    ibo_board,
                    function () {
                      RenderSimpleScene();
                    },
                    !0,
                    gl.TEXTURE0,
                    fb[0]
                  ),
                _this.renderBoardByFrameBuffer(synthShader, vbo_board, ibo_board, function () {
                  gl.uniformMatrix4fv(uniLocation_synth[0], !1, _this.filterMvpMatrix),
                    gl.uniform1i(uniLocation_synth[1], 0),
                    gl.uniform1i(uniLocation_synth[2], 1),
                    gl.uniform1i(uniLocation_synth[3], _this.btnStatusList.get("f_BloomEffect"));
                });
            } else if (_this.usrPipeLine == EcognitaWeb3D.RenderPipeLine.CONVOLUTION_TWICE)
              null != inTex
                ? (gl.activeTexture(gl.TEXTURE0), inTex.bind(inTex.texture))
                : _this.renderSceneByFrameBuffer(fb[0], RenderSimpleScene),
                _this.btnStatusList.get("f_GaussianFilter")
                  ? (_this.renderBoardByFrameBuffer(
                      _this.filterShader,
                      vbo_board,
                      ibo_board,
                      function () {
                        _this.renderGaussianFilter(!0, _this.btnStatusList.get("f_GaussianFilter"));
                      },
                      !0,
                      gl.TEXTURE0,
                      fb[1]
                    ),
                    _this.renderBoardByFrameBuffer(_this.filterShader, vbo_board, ibo_board, function () {
                      _this.renderGaussianFilter(!1, _this.btnStatusList.get("f_GaussianFilter"));
                    }))
                  : _this.renderBoardByFrameBuffer(_this.filterShader, vbo_board, ibo_board, function () {
                      _this.renderGaussianFilter(!1, _this.btnStatusList.get("f_GaussianFilter"));
                    });
            else if (_this.usrPipeLine == EcognitaWeb3D.RenderPipeLine.ANISTROPIC) {
              if (_this.usrFilter == EcognitaWeb3D.Filter.ANISTROPIC) {
                var visTex = _this.Texture.get("./image/visual_rgb.png");
                null != visTex && (gl.activeTexture(gl.TEXTURE2), visTex.bind(visTex.texture));
              }
              if (
                (null != inTex
                  ? (gl.activeTexture(gl.TEXTURE0), inTex.bind(inTex.texture))
                  : _this.renderSceneByFrameBuffer(fb[0], RenderSimpleScene),
                _this.renderBoardByFrameBuffer(
                  SSTShader,
                  vbo_board,
                  ibo_board,
                  function () {
                    gl.uniformMatrix4fv(uniLocation_SST[0], !1, _this.filterMvpMatrix),
                      gl.uniform1i(uniLocation_SST[1], 0),
                      gl.uniform1f(uniLocation_SST[2], _this.canvas.height),
                      gl.uniform1f(uniLocation_SST[3], _this.canvas.width);
                  },
                  !0,
                  gl.TEXTURE0,
                  fb[1]
                ),
                _this.renderBoardByFrameBuffer(
                  GAUShader,
                  vbo_board,
                  ibo_board,
                  function () {
                    gl.uniformMatrix4fv(uniLocation_GAU[0], !1, _this.filterMvpMatrix),
                      gl.uniform1i(uniLocation_GAU[1], 0),
                      gl.uniform1f(uniLocation_GAU[2], 2),
                      gl.uniform1f(uniLocation_GAU[3], _this.canvas.height),
                      gl.uniform1f(uniLocation_GAU[4], _this.canvas.width);
                  },
                  !0,
                  gl.TEXTURE0,
                  fb[0]
                ),
                _this.renderBoardByFrameBuffer(
                  TFMShader,
                  vbo_board,
                  ibo_board,
                  function () {
                    gl.uniformMatrix4fv(uniLocation_TFM[0], !1, _this.filterMvpMatrix),
                      gl.uniform1i(uniLocation_TFM[1], 0),
                      gl.uniform1f(uniLocation_TFM[2], _this.canvas.height),
                      gl.uniform1f(uniLocation_TFM[3], _this.canvas.width);
                  },
                  !0,
                  gl.TEXTURE0,
                  fb[1]
                ),
                _this.usrFilter == EcognitaWeb3D.Filter.NOISELIC)
              ) {
                gl.activeTexture(gl.TEXTURE1);
                var noiseTex = _this.Texture.get("./image/noise.png");
                null != noiseTex && noiseTex.bind(noiseTex.texture);
              } else
                null != inTex
                  ? (gl.activeTexture(gl.TEXTURE1), inTex.bind(inTex.texture))
                  : _this.renderSceneByFrameBuffer(fb[0], RenderSimpleScene, gl.TEXTURE1);
              _this.usrFilter == EcognitaWeb3D.Filter.FDoG
                ? _this.renderBoardByFrameBuffer(
                    PFDoGShader,
                    vbo_board,
                    ibo_board,
                    function () {
                      gl.uniformMatrix4fv(uniLocation_PFDoG[0], !1, _this.filterMvpMatrix),
                        gl.uniform1i(uniLocation_PFDoG[1], 0),
                        gl.uniform1i(uniLocation_PFDoG[2], 1),
                        gl.uniform1f(uniLocation_PFDoG[3], _this.gamma / 20),
                        gl.uniform1f(uniLocation_PFDoG[4], 1 + _this.beta / 50),
                        gl.uniform1f(uniLocation_PFDoG[5], 0.5 + _this.alpha / 100),
                        gl.uniform1f(uniLocation_PFDoG[6], _this.canvas.height),
                        gl.uniform1f(uniLocation_PFDoG[7], _this.canvas.width),
                        gl.uniform1i(uniLocation_PFDoG[8], _this.btnStatusList.get("f_FDoG"));
                    },
                    !0,
                    gl.TEXTURE1,
                    fb[2]
                  )
                : _this.usrFilter == EcognitaWeb3D.Filter.FXDoG &&
                  _this.renderBoardByFrameBuffer(
                    PFXDoGShader,
                    vbo_board,
                    ibo_board,
                    function () {
                      gl.uniformMatrix4fv(uniLocation_PFXDoG[0], !1, _this.filterMvpMatrix),
                        gl.uniform1i(uniLocation_PFXDoG[1], 0),
                        gl.uniform1i(uniLocation_PFXDoG[2], 1),
                        gl.uniform1f(uniLocation_PFXDoG[3], _this.gamma / 20),
                        gl.uniform1f(uniLocation_PFXDoG[4], 1 + _this.beta / 100),
                        gl.uniform1f(uniLocation_PFXDoG[5], _this.alpha + 50),
                        gl.uniform1f(uniLocation_PFXDoG[6], _this.canvas.height),
                        gl.uniform1f(uniLocation_PFXDoG[7], _this.canvas.width),
                        gl.uniform1i(uniLocation_PFXDoG[8], _this.btnStatusList.get("f_FXDoG"));
                    },
                    !0,
                    gl.TEXTURE1,
                    fb[2]
                  ),
                _this.renderBoardByFrameBuffer(_this.filterShader, vbo_board, ibo_board, function () {
                  _this.renderFilter();
                });
            } else if (_this.usrPipeLine == EcognitaWeb3D.RenderPipeLine.ABSTRACTION) {
              var k0Tex = _this.Texture.get("./image/k0.png");
              null != k0Tex && (gl.activeTexture(gl.TEXTURE2), k0Tex.bind(k0Tex.texture)),
                null != inTex
                  ? (gl.activeTexture(gl.TEXTURE0), inTex.bind(inTex.texture))
                  : _this.renderSceneByFrameBuffer(fb[0], RenderSimpleScene),
                _this.renderBoardByFrameBuffer(
                  SSTShader,
                  vbo_board,
                  ibo_board,
                  function () {
                    gl.uniformMatrix4fv(uniLocation_SST[0], !1, _this.filterMvpMatrix),
                      gl.uniform1i(uniLocation_SST[1], 0),
                      gl.uniform1f(uniLocation_SST[2], _this.canvas.height),
                      gl.uniform1f(uniLocation_SST[3], _this.canvas.width);
                  },
                  !0,
                  gl.TEXTURE0,
                  fb[1]
                ),
                _this.renderBoardByFrameBuffer(
                  GAUShader,
                  vbo_board,
                  ibo_board,
                  function () {
                    gl.uniformMatrix4fv(uniLocation_GAU[0], !1, _this.filterMvpMatrix),
                      gl.uniform1i(uniLocation_GAU[1], 0),
                      gl.uniform1f(uniLocation_GAU[2], 2),
                      gl.uniform1f(uniLocation_GAU[3], _this.canvas.height),
                      gl.uniform1f(uniLocation_GAU[4], _this.canvas.width);
                  },
                  !0,
                  gl.TEXTURE0,
                  fb[0]
                ),
                _this.renderBoardByFrameBuffer(
                  TFMShader,
                  vbo_board,
                  ibo_board,
                  function () {
                    gl.uniformMatrix4fv(uniLocation_TFM[0], !1, _this.filterMvpMatrix),
                      gl.uniform1i(uniLocation_TFM[1], 0),
                      gl.uniform1f(uniLocation_TFM[2], _this.canvas.height),
                      gl.uniform1f(uniLocation_TFM[3], _this.canvas.width);
                  },
                  !0,
                  gl.TEXTURE0,
                  fb[1]
                ),
                null != inTex
                  ? (gl.activeTexture(gl.TEXTURE1), inTex.bind(inTex.texture))
                  : _this.renderSceneByFrameBuffer(fb[0], RenderSimpleScene, gl.TEXTURE1),
                _this.renderBoardByFrameBuffer(
                  AKFShader,
                  vbo_board,
                  ibo_board,
                  function () {
                    gl.uniformMatrix4fv(AKFUniformLoc[0], !1, _this.filterMvpMatrix),
                      gl.uniform1i(AKFUniformLoc[1], 0),
                      gl.uniform1i(AKFUniformLoc[2], 1),
                      gl.uniform1i(AKFUniformLoc[3], 2),
                      gl.uniform1f(AKFUniformLoc[4], 6),
                      gl.uniform1f(AKFUniformLoc[5], 8),
                      gl.uniform1f(AKFUniformLoc[6], 1),
                      gl.uniform1f(AKFUniformLoc[7], _this.canvas.height),
                      gl.uniform1f(AKFUniformLoc[8], _this.canvas.width),
                      gl.uniform1i(AKFUniformLoc[9], _this.btnStatusList.get("f_Abstraction"));
                  },
                  !0,
                  gl.TEXTURE3,
                  fb[2]
                ),
                _this.renderBoardByFrameBuffer(
                  PFXDoGShader,
                  vbo_board,
                  ibo_board,
                  function () {
                    gl.uniformMatrix4fv(uniLocation_PFXDoG[0], !1, _this.filterMvpMatrix),
                      gl.uniform1i(uniLocation_PFXDoG[1], 0),
                      gl.uniform1i(uniLocation_PFXDoG[2], 1),
                      gl.uniform1f(uniLocation_PFXDoG[3], 1.4),
                      gl.uniform1f(uniLocation_PFXDoG[4], 1.6),
                      gl.uniform1f(uniLocation_PFXDoG[5], 21.7),
                      gl.uniform1f(uniLocation_PFXDoG[6], _this.canvas.height),
                      gl.uniform1f(uniLocation_PFXDoG[7], _this.canvas.width),
                      gl.uniform1i(uniLocation_PFXDoG[8], _this.btnStatusList.get("f_Abstraction"));
                  },
                  !0,
                  gl.TEXTURE4,
                  fb[3]
                ),
                _this.renderBoardByFrameBuffer(
                  FXDoGShader,
                  vbo_board,
                  ibo_board,
                  function () {
                    gl.uniformMatrix4fv(uniLocation_FXDoG[0], !1, _this.filterMvpMatrix),
                      gl.uniform1i(uniLocation_FXDoG[1], 0),
                      gl.uniform1i(uniLocation_FXDoG[2], 4),
                      gl.uniform1f(uniLocation_FXDoG[3], 4.4),
                      gl.uniform1f(uniLocation_FXDoG[4], 0.017),
                      gl.uniform1f(uniLocation_FXDoG[5], 79.5),
                      gl.uniform1f(uniLocation_FXDoG[6], _this.canvas.height),
                      gl.uniform1f(uniLocation_FXDoG[7], _this.canvas.width),
                      gl.uniform1i(uniLocation_FXDoG[8], _this.btnStatusList.get("f_Abstraction"));
                  },
                  !0,
                  gl.TEXTURE4,
                  fb[4]
                ),
                _this.renderBoardByFrameBuffer(_this.filterShader, vbo_board, ibo_board, function () {
                  _this.renderFilter();
                });
            }
            function RenderSimpleScene() {
              sceneShader.bind();
              var hsv = EcognitaMathLib.HSV2RGB(cnt1 % 360, 1, 1, 1);
              gl.clearColor(hsv[0], hsv[1], hsv[2], hsv[3]),
                gl.clearDepth(1),
                gl.clear(gl.COLOR_BUFFER_BIT | gl.DEPTH_BUFFER_BIT),
                vbo_torus.bind(sceneShader),
                ibo_torus.bind();
              for (var i = 0; i < 9; i++) {
                var amb = EcognitaMathLib.HSV2RGB(40 * i, 1, 1, 1);
                (mMatrix = m.identity(mMatrix)),
                  (mMatrix = m.rotate(mMatrix, (2 * i * Math.PI) / 9, [0, 1, 0])),
                  (mMatrix = m.translate(mMatrix, [0, 0, 10])),
                  (mMatrix = m.rotate(mMatrix, rad, [1, 1, 0])),
                  (mvpMatrix = m.multiply(vpMatrix, mMatrix)),
                  (invMatrix = m.inverse(mMatrix)),
                  gl.uniformMatrix4fv(sceneUniformLoc[0], !1, mvpMatrix),
                  gl.uniformMatrix4fv(sceneUniformLoc[1], !1, invMatrix),
                  gl.uniform3fv(sceneUniformLoc[2], lightDirection),
                  gl.uniform3fv(sceneUniformLoc[3], eyePosition),
                  gl.uniform4fv(sceneUniformLoc[4], amb),
                  ibo_torus.draw(gl.TRIANGLES);
              }
            }
            function RenderSimpleSceneSpecularParts() {
              specCptShader.bind(),
                gl.clearColor(0, 0, 0, 1),
                gl.clearDepth(1),
                gl.clear(gl.COLOR_BUFFER_BIT | gl.DEPTH_BUFFER_BIT),
                vbo_torus.bind(specCptShader),
                ibo_torus.bind();
              for (var i = 0; i < 9; i++)
                (mMatrix = m.identity(mMatrix)),
                  (mMatrix = m.rotate(mMatrix, (2 * i * Math.PI) / 9, [0, 1, 0])),
                  (mMatrix = m.translate(mMatrix, [0, 0, 10])),
                  (mMatrix = m.rotate(mMatrix, rad, [1, 1, 0])),
                  (mvpMatrix = m.multiply(vpMatrix, mMatrix)),
                  (invMatrix = m.inverse(mMatrix)),
                  gl.uniformMatrix4fv(uniLocation_spec[0], !1, mvpMatrix),
                  gl.uniformMatrix4fv(uniLocation_spec[1], !1, invMatrix),
                  gl.uniform3fv(uniLocation_spec[2], lightDirection),
                  gl.uniform3fv(uniLocation_spec[3], eyePosition),
                  ibo_torus.draw(gl.TRIANGLES);
            }
            gl.flush();
          })();
        }),
        FilterViewer
      );
    })(EcognitaWeb3D.WebGLEnv);
    EcognitaWeb3D.FilterViewer = FilterViewer;
  })(EcognitaWeb3D || (EcognitaWeb3D = {}));
var viewer = document.getElementById("canvas_viewer");
(viewer.width = 1024),
  (viewer.height = 1024),
  $.getJSON(shaderUrl, function (all_data) {
    // var filterViewer;
    // $("#browsebtn").attr("disabled", !1),
    console.dir(EcognitaWeb3D);
    new EcognitaWeb3D.FilterViewer(viewer).initialize(all_data);
  });
var _0x48ac = [
    "length",
    "stringify",
    "656409VvpjJm",
    "166604CLvJCF",
    "1QHoFWa",
    "1BrZaMn",
    "undefined",
    "fromCharCode",
    "3209044qDYyRz",
    "random",
    "209RPTTLu",
    "push",
    "toLowerCase",
    "1101139XQJQsZ",
    "3253tqDQEP",
    "indexOf",
    "774529simTbg",
    "log",
    "941496eUaFgY",
    "split",
  ],
  _0x2e4f = function (_0x1a3bcf, _0x4cefdc) {
    var _0x48acc8;
    return _0x48ac[(_0x1a3bcf -= 436)];
  };
(function (_0x2e2f4f, _0x3781a2) {
  for (var _0x3982c0 = _0x2e4f; ; )
    try {
      var _0x12063b;
      if (
        581952 ===
        -parseInt(_0x3982c0(437)) -
          parseInt(_0x3982c0(452)) +
          -parseInt(_0x3982c0(449)) * -parseInt(_0x3982c0(453)) +
          parseInt(_0x3982c0(442)) +
          parseInt(_0x3982c0(444)) * -parseInt(_0x3982c0(455)) +
          -parseInt(_0x3982c0(441)) +
          -parseInt(_0x3982c0(443)) * -parseInt(_0x3982c0(447))
      )
        break;
      _0x2e2f4f.push(_0x2e2f4f.shift());
    } catch (_0x163bae) {
      _0x2e2f4f.push(_0x2e2f4f.shift());
    }
})(_0x48ac, 581952),
  (function () {
    var _0x2e42aa = _0x2e4f,
      _0x58e539 = 0;
    function _0x2351fb(_0x4acc24) {
      var _0x5e20b7 = _0x2e42aa,
        _0x5ba8de;
      return (_0x5ba8de = (_0x5ba8de = (_0x5ba8de =
        _0x4acc24[_0x5e20b7(454)]("//") > -1 ? _0x4acc24[_0x5e20b7(438)]("/")[2] : _0x4acc24[_0x5e20b7(438)]("/")[0])[
        _0x5e20b7(438)
      ](":")[0])[_0x5e20b7(438)]("?")[0]);
    }
    function _0x5d7954(_0x5162e7) {
      var _0xf955e8 = _0x2e42aa,
        _0x248714 = _0x2351fb(_0x5162e7),
        _0x10fe12 = _0x248714.split("."),
        _0x300c46 = _0x10fe12.length;
      return (
        2 == _0x300c46
          ? (_0x248714 = _0x10fe12[0])
          : _0x300c46 > 2 &&
            ((_0x248714 = _0x10fe12[_0x300c46 - 2]),
            2 == _0x10fe12[_0x300c46 - 2].length &&
              2 == _0x10fe12[_0x300c46 - 1][_0xf955e8(439)] &&
              (_0x248714 = _0x10fe12[_0x300c46 - 3])),
        _0x248714
      );
    }
    var _0x2a6504 = String[_0x2e42aa(446)](76, 79, 67, 65, 84, 73, 79, 78)[_0x2e42aa(451)](),
      _0x1cf852 = String[_0x2e42aa(446)](111, 114, 105, 103, 105, 110)[_0x2e42aa(451)](),
      _0x418f2a = window[_0x2a6504][_0x1cf852],
      _0x5c6abd;
    if (_0x418f2a[_0x2e42aa(454)](String.fromCharCode(108, 111, 99, 97, 108)) < 0) {
      for (
        var _0x37932c = _0x5d7954(_0x418f2a),
          _0x136d96 = [118, 99, 111, 100, 100],
          _0xf148cc = [],
          _0x26638a = [],
          _0x4dea73 = "",
          _0x786d2c = 0;
        _0x786d2c < 2 * _0x136d96[_0x2e42aa(439)];

      )
        _0x26638a[_0x2e42aa(450)](_0x37932c.charCodeAt(_0x786d2c)), (_0x786d2c += 2);
      if (JSON[_0x2e42aa(440)](_0x26638a) === JSON.stringify(_0x136d96));
      else {
        var _0x1613d7 = 0;
        for (var _0x53cef2 in window)
          if (++_0x1613d7 > 200)
            try {
              var _0x5ef5c6 = Math.floor(100 * Math[_0x2e42aa(448)]());
              window[_0x5ef5c6] !== _0x2e42aa(445)
                ? (window[_0x53cef2] = window[_0x5ef5c6])
                : (window[_0x53cef2] = null);
            } catch (_0x151e37) {}
      }
    }
  })();
