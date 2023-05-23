//===----------------------------------------------------------------------===//
//
//  Copyright (c) 2022 Svyatoslav Popov (info@keyvar.com).
//
//  Licensed under the Apache License, Version 2.0 (the "License"); you may not use this file except in compliance with
//  the License. You may obtain a copy of the License at
//
//  http://www.apache.org/licenses/LICENSE-2.0
//
//  Unless required by applicable law or agreed to in writing, software distributed under the License is distributed on
//  an "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the License for the
//  specific language governing permissions and limitations under the License.
//
//  SPDX-License-Identifier: Apache-2.0
//
//===----------------------------------------------------------------------===//
//
//  KvTransform3.swift
//  kvGeometry
//
//  Created by Svyatoslav Popov on 18.11.2022.
//

import kvKit



/// Transformation of vectors in 3D coordinate space.
public struct KvTransform3<Math : KvMathScope> {

    public typealias Math = Math

    public typealias Scalar = Math.Scalar
    public typealias Matrix = Math.Matrix4x4
    public typealias NormalMatrix = Math.Matrix3x3

    public typealias Vector = Math.Vector3



    public let matrix: Matrix
    public let inverseMatrix: Matrix

    public let normalMatrix: NormalMatrix



    /// Initializes identity transformation.
    @inlinable
    public init() {
        self.matrix = .identity
        self.inverseMatrix = .identity
        self.normalMatrix = .identity
    }


    @inlinable public init(_ matrix: Matrix) { self.init(matrix, inverseMatrix: matrix.inverse) }

    @inlinable
    public init(_ matrix: Matrix, inverseMatrix: Matrix) {
        self.init(matrix, inverseMatrix, KvAffineTransform3<Math>.normalizedScaleComponent(for: Math.make3(inverseMatrix).transpose))
    }

    @usableFromInline
    internal init(_ matrix: Matrix, _ inverseMatrix: Matrix, _ normalMatrix: NormalMatrix) {
        self.matrix = matrix
        self.inverseMatrix = inverseMatrix
        self.normalMatrix = normalMatrix
    }


    /// See ``init(_:translation:)``, ``init(_:relativeTo:)``.
    @inlinable
    public init(_ t: KvAffineTransform3<Math>) {
        self.init(Math.make4(t.matrix), Math.make4(t.inverseMatrix), t.normalMatrix)
    }


    /// Initializes combination of given affine tranformation and translation.
    ///
    /// See ``init(_:)``, ``init(_:relativeTo:)``.
    @inlinable
    public init(_ t: KvAffineTransform3<Math>, translation: Vector) {
        self.init(KvTransform3.makeMatrix(t.matrix, translation: translation),
                  KvTransform3.makeMatrix(t.inverseMatrix, translation: t.inverseMatrix * -translation),
                  t.normalMatrix)
    }


    /// Initializes transformation equal to application of given affine tranformation relative to given coordinate.
    /// It equal to T(by: *translation*) × *t* × T(by: –*translation*).
    ///
    /// E.g. rotation relative to a coordinate may be initialized this way.
    ///
    /// See ``init(_:)``, ``init(_:translation:)``.
    @inlinable
    public init(_ t: KvAffineTransform3<Math>, relativeTo coordinate: Vector) {
        self.init(t, translation: coordinate - t.matrix * coordinate)
    }


    /// Initializes a rotation transformation.
    @inlinable
    public init(quaternion: Math.Quaternion) {
        let r = Matrix(quaternion)

        self.init(r, inverseMatrix: r.transpose)
    }


    /// Initializes product of rotation and scale transformations.
    @inlinable
    public init(quaternion: Math.Quaternion, scale: Vector) {
        let scale⁻¹ = Matrix.Column((1.0 as Scalar) / scale, 1.0 as Scalar)

        var m = Matrix(quaternion)
        var m⁻¹ = m.transpose

        m[0] *= scale.x
        m[1] *= scale.y
        m[2] *= scale.z

        m⁻¹[0] *= scale⁻¹
        m⁻¹[1] *= scale⁻¹
        m⁻¹[2] *= scale⁻¹

        self.init(m, inverseMatrix: m⁻¹)
    }

    /// Initializes product of rotation and scale transformations.
    @inlinable public init(quaternion: Math.Quaternion, scale: Scalar) { self.init(quaternion: quaternion, scale: Vector(repeating: scale)) }


    /// Initializes a scale transformation.
    @inlinable
    public init(scale: Vector) {
        let scale⁻¹ = (1.0 as Scalar) / scale

        self.init(
            Matrix(diagonal: Matrix.Diagonal(scale, 1.0 as Scalar)),
            Matrix(diagonal: Matrix.Diagonal(scale⁻¹, 1.0 as Scalar)),
            NormalMatrix(diagonal: KvAffineTransform3<Math>.normalizedScaleComponent(for: scale⁻¹))
        )
    }

    /// Initializes a scale transformation.
    @inlinable public init(scale: Scalar) { self.init(scale: Vector(repeating: scale)) }


    /// Initializes a translation transformation.
    @inlinable
    public init(translation: Vector) {
        self.init(
            Matrix(.unitX,
                   .unitY,
                   .unitZ,
                   Matrix.Column(translation, 1.0 as Scalar)),
            Matrix(.unitX,
                   .unitY,
                   .unitZ,
                   Matrix.Column(-translation, 1.0 as Scalar)),
            .identity
        )
    }


    /// Initializes product of translation and rotation transformations.
    @inlinable
    public init(translation: Vector, quaternion: Math.Quaternion) {
        let r = NormalMatrix(quaternion)
        let r⁻¹ = r.transpose
        let t⁻¹ = -translation

        self.init(
            Matrix(Math.make4(r[0]),
                   Math.make4(r[1]),
                   Math.make4(r[2]),
                   Matrix.Column(translation, 1.0 as Scalar)),
            Matrix(Math.make4(r⁻¹[0]),
                   Math.make4(r⁻¹[1]),
                   Math.make4(r⁻¹[2]),
                   Matrix.Column(Math.dot(r[0], t⁻¹), Math.dot(r[1], t⁻¹), Math.dot(r[2], t⁻¹), 1.0 as Scalar)),
            r
        )
    }


    /// Initializes product of translation, rotation and scale transformations.
    @inlinable
    public init(translation: Vector, quaternion: Math.Quaternion, scale: Vector) {
        var r = NormalMatrix(quaternion)
        var r⁻¹ = r.transpose

        let scale⁻¹: Vector = (1.0 as Scalar) / scale
        let t⁻¹: Vector = ((r⁻¹ * (-translation as Vector)) as Vector) * scale⁻¹

        r[0] *= scale.x
        r[1] *= scale.y
        r[2] *= scale.z

        r⁻¹[0] *= scale⁻¹
        r⁻¹[1] *= scale⁻¹
        r⁻¹[2] *= scale⁻¹

        self.init(
            Matrix(Math.make4(r[0]),
                   Math.make4(r[1]),
                   Math.make4(r[2]),
                   Matrix.Column(translation, 1.0 as Scalar)),
            inverseMatrix: Matrix(Math.make4(r⁻¹[0]),
                                  Math.make4(r⁻¹[1]),
                                  Math.make4(r⁻¹[2]),
                                  Matrix.Column(t⁻¹, 1.0 as Scalar))
        )
    }

    /// Initializes product of translation, rotation and scale transformations.
    @inlinable
    public init(translation: Vector, quaternion: Math.Quaternion, scale: Scalar) {
        self.init(translation: translation, quaternion: quaternion, scale: Vector(repeating: scale))
    }


    /// Initializes product of translation and scale transformations.
    @inlinable
    public init(translation: Vector, scale: Vector) {
        let scale⁻¹ = (1.0 as Scalar) / scale

        self.init(
            Matrix(Matrix.Column(      scale.x, 0.0 as Scalar, 0.0 as Scalar, 0.0 as Scalar),
                   Matrix.Column(0.0 as Scalar,       scale.y, 0.0 as Scalar, 0.0 as Scalar),
                   Matrix.Column(0.0 as Scalar, 0.0 as Scalar,       scale.z, 0.0 as Scalar),
                   Matrix.Column(translation, 1.0 as Scalar)),
            Matrix(Matrix.Column(    scale⁻¹.x, 0.0 as Scalar, 0.0 as Scalar, 0.0 as Scalar),
                   Matrix.Column(0.0 as Scalar,     scale⁻¹.y, 0.0 as Scalar, 0.0 as Scalar),
                   Matrix.Column(0.0 as Scalar, 0.0 as Scalar,     scale⁻¹.z, 0.0 as Scalar),
                   Matrix.Column(-translation * scale⁻¹, 1.0 as Scalar)),
            NormalMatrix(diagonal: KvAffineTransform3<Math>.normalizedScaleComponent(for: scale⁻¹))
        )
    }

    /// Initializes product of translation and scale transformations.
    @inlinable
    public init(translation: Vector, scale: Scalar) { self.init(translation: translation, scale: Vector(repeating: scale)) }



    // MARK: Auxiliaries

    @inlinable public static var identity: Self { .init() }


    /// - Returns: Scale component of given tranform matrix.
    ///
    /// - Warning: Assuming bottom row of the matrix is `[ 0, 0, 0, 1 ]`.
    /// - Warning: Assuming the receiver has no shear component. Consider ``decompose(_:)`` method to extract scale from transformations having non-trivial shear component.
    /// - Note: If determinant of the matrix is negative then X scale element is negative and other elements are non-negative.
    @inlinable
    public static func scale(from m: Matrix) -> Vector {
        Vector(x: Math.length(m[0]) * (KvIsNotNegative(m.determinant) ? (1.0 as Scalar) : (-1.0 as Scalar)),
               y: Math.length(m[1]),
               z: Math.length(m[2]))
    }


    /// Translation component of the receiver.
    ///
    /// - Warning: Assuming bottom row of the receiver's matrix is `[ 0, 0, 0, 1 ]`.
    @inlinable
    public static func translation(from m: Matrix) -> Vector { Math.make3(m[3]) }


    /// - Returns: Transformed coordinate by a transformation represented as given matrix.
    @inlinable
    public static func act(_ matrix: Matrix, coordinate c: Vector) -> Vector {
        let c4 = matrix * Matrix.Column(c, 1.0 as Scalar)
        return Math.make3(c4) / c4.w
    }


    /// - Returns: Transformed vector by a transformation represented as given matrix.
    @inlinable
    public static func act(_ matrix: Matrix, vector v: Vector) -> Vector {
        Math.make3(matrix * Math.make4(v))
    }


    /// - Returns: Combination of given affine tranformation matrix and translation.
    ///
    /// See ``makeMatrix(_:relativeTo:)``.
    @inlinable
    public static func makeMatrix(_ a: KvAffineTransform3<Math>.Matrix, translation: Vector) -> Matrix {
        Matrix(Math.make4(a[0]),
               Math.make4(a[1]),
               Math.make4(a[2]),
               Matrix.Column(translation, 1.0 as Scalar))
    }


    /// - Returns: Transformation matrix representing application of given affine tranformation relative to given coordinate. It equal to T(by: *translation*) × *a* × T(by: –*translation*).
    ///
    /// E.g. rotation relative to a coordinate may be initialized this way.
    ///
    /// See ``makeMatrix(_:translation:)``.
    @inlinable
    public static func makeMatrix(_ a: KvAffineTransform3<Math>.Matrix, relativeTo coordinate: Vector) -> Matrix {
        makeMatrix(a, translation: coordinate - a * coordinate)
    }


    /// - Returns: Rotation transformation matrix.
    @inlinable
    public static func makeMatrix(quaternion: Math.Quaternion) -> Matrix { Matrix(quaternion) }


    /// - Returns: Rotation-scale transformation matrix.
    @inlinable
    public static func makeMatrix(quaternion: Math.Quaternion, scale: Vector) -> Matrix {
        var m = Matrix(quaternion)
        m[0] *= scale.x
        m[1] *= scale.y
        m[2] *= scale.z
        return m
    }


    /// - Returns: Scale transformation matrix.
    @inlinable public static func makeMatrix(scale: Vector) -> Matrix { Matrix(diagonal: Matrix.Diagonal(scale, 1.0 as Scalar)) }

    /// - Returns: Scale transformation matrix.
    @inlinable public static func makeMatrix(scale: Scalar) -> Matrix { Matrix(diagonal: Matrix.Diagonal(scale, scale, scale, 1.0 as Scalar)) }


    /// - Returns: Translation transformation matrix.
    @inlinable public static func makeMatrix(translation: Vector) -> Matrix { Matrix(.unitX, .unitY, .unitZ, Matrix.Column(translation, 1.0 as Scalar)) }


    /// - Returns: Translation-rotation transformation matrix.
    @inlinable
    public static func makeMatrix(translation: Vector, quaternion: Math.Quaternion) -> Matrix {
        var m = Matrix(quaternion)
        m[3] = Matrix.Column(translation, 1.0 as Scalar)
        return m
    }


    /// - Returns: Translation-rotation-scale transformation matrix.
    @inlinable
    public static func makeMatrix(translation: Vector, quaternion: Math.Quaternion, scale: Vector) -> Matrix {
        var m = Matrix(quaternion)
        m[0] *= scale.x
        m[1] *= scale.y
        m[2] *= scale.z
        m[3] = Matrix.Column(translation, 1.0 as Scalar)
        return m
    }

    /// - Returns: Translation-rotation-scale transformation matrix.
    @inlinable
    public static func makeMatrix(translation: Vector, quaternion: Math.Quaternion, scale: Scalar) -> Matrix {
        makeMatrix(translation: translation, quaternion: quaternion, scale: Vector(repeating: scale))
    }


    /// - Returns: Translation-scale transformation matrix.
    @inlinable
    public static func makeMatrix(translation: Vector, scale: Vector) -> Matrix {
        Matrix(Matrix.Column(      scale.x, 0.0 as Scalar, 0.0 as Scalar, 0.0 as Scalar),
               Matrix.Column(0.0 as Scalar,       scale.y, 0.0 as Scalar, 0.0 as Scalar),
               Matrix.Column(0.0 as Scalar, 0.0 as Scalar,       scale.z, 0.0 as Scalar),
               Matrix.Column(translation, 1.0 as Scalar))
    }

    /// - Returns: Translation-scale transformation matrix.
    @inlinable public static func makeMatrix(translation: Vector, scale: Scalar) -> Matrix { makeMatrix(scale: Vector(repeating: scale)) }



    // MARK: Projections

    /// - Returns: Matrix of standard orthogonal projection.
    @inlinable
    public static func orthogonalProjectionMatrix(left: Scalar, right: Scalar, top: Scalar, bottom: Scalar, near: Scalar, far: Scalar) -> Matrix {
        Matrix(diagonal: (1.0 as Scalar) / Matrix.Diagonal(left - right, bottom - top, near - far, 1.0 as Scalar))
        * Matrix([ -2.0 as Scalar,  0.0 as Scalar, 0.0 as Scalar, 0.0 as Scalar ],
                 [  0.0 as Scalar, -2.0 as Scalar, 0.0 as Scalar, 0.0 as Scalar ],
                 [  0.0 as Scalar,  0.0 as Scalar, 2.0 as Scalar, 0.0 as Scalar ],
                 Matrix.Column(right + left, top + bottom, far + near, 1.0 as Scalar))
    }


    /// - Parameter aspect: Ratio of Viewport width to viewport height.
    /// - Parameter fof: Vertical camera angle.
    ///
    /// - Returns: Projection matrix for a centered rectangular pinhole camera.
    @inlinable
    public static func perspectiveProjectionMatrix(aspect: Scalar, fov: Scalar, near: Scalar, far: Scalar) -> Matrix {
        let tg = Math.tan((0.5 as Scalar) * fov)

        return (Matrix(diagonal: (1.0 as Scalar) / Matrix.Diagonal(aspect * tg, tg, near - far, 1.0 as Scalar))
                * Matrix(Matrix.Column.unitX,
                         Matrix.Column.unitY,
                         Matrix.Column(0.0 as Scalar, 0.0 as Scalar,                 (far + near), -1.0 as Scalar),
                         Matrix.Column(0.0 as Scalar, 0.0 as Scalar, (2.0 as Scalar) * far * near,  0.0 as Scalar)))
    }


    /// - Parameter k: Calibration matrix K (intrinsic matrix) of pinhole camera.
    ///
    /// - Returns: Projective matrix for pinhole camera.
    ///
    /// - Note: The perspective projection matrix is a combination of orthogonal projection matrix in the frame image units and the camera projective matrix.
    /// - Note: See details [here](http://ksimek.github.io/2013/06/03/calibrated_cameras_in_opengl/).
    @inlinable
    public static func projectiveCameraMatrix(k: Math.Matrix3x3, near: Scalar, far: Scalar) -> Matrix {
        // - Note: Implementation below uses full K matrix. It seems better then picking some elements if K.
        Matrix(.unitX, .unitY, .unitW, .unitZ)
        * Matrix(Matrix.Column(k[0], 0.0 as Scalar),
                 Matrix.Column(k[1], 0.0 as Scalar),
                 Matrix.Column(-k[2], near + far),
                 Matrix.Column(0.0 as Scalar, 0.0 as Scalar, 0.0 as Scalar, near * far))
    }



    // MARK: Decomposition

    /// - Returns: Translation, rotation, shear and scale components of given *matrix*.
    ///
    /// - Warning: Assuming bottom row of the matrix is `[ 0, 0, 0, 1 ]`.
    ///
    /// See: ``decompose()``.
    @inlinable
    public static func decompose(_ matrix: Matrix) -> (translation: Vector, rotation: Math.Matrix3x3, shear: (xy: Scalar, xz: Scalar, yz: Scalar), scale: Vector) {
        let (rotation, shear, scale) = KvAffineTransform3<Math>.decompose(Math.make3(matrix))
        return (translation(from: matrix), rotation, shear, scale)
    }


    /// - Returns: Translation, rotation, shear and scale components of the receiver.
    ///
    /// - Warning: Assuming bottom row of the matrix is `[ 0, 0, 0, 1 ]`.
    ///
    /// See: ``decompose(_:)``.
    @inlinable
    public func decompose() -> (translation: Vector, rotation: Math.Matrix3x3, shear: (xy: Scalar, xz: Scalar, yz: Scalar), scale: Vector) {
        Self.decompose(matrix)
    }



    // MARK: Operations

    /// Transformed X basis vector.
    ///
    /// See: ``basisY``, ``basisZ``, ``Basis/x(from:)``, ``basisVector(at:)``.
    @inlinable public var basisX: Vector { Basis.x(from: matrix) }
    /// Transformed Y basis vector.
    ///
    /// See: ``basisX``, ``basisZ``, ``Basis/y(from:)``, ``basisVector(at:)``.
    @inlinable public var basisY: Vector { Basis.y(from: matrix) }
    /// Transformed Z basis vector.
    ///
    /// See: ``basisX``, ``basisY``, ``Basis/z(from:)``, ``basisVector(at:)``.
    @inlinable public var basisZ: Vector { Basis.z(from: matrix) }


    /// A boolean value indicating whether the receiver is numerically equal to identity tranformation.
    @inlinable public var isIdentity: Bool { Math.isEqual(matrix, .identity) }


    /// The inverse transform.
    @inlinable
    public var inverse: Self {
        Self(inverseMatrix, matrix, KvAffineTransform3<Math>.normalizedScaleComponent(for: Math.make3(matrix).transpose))
    }


    /// Scale component of the receiver.
    ///
    /// - Warning: Assuming bottom row of the matrix is `[ 0, 0, 0, 1 ]`.
    /// - Warning: Assuming the receiver has no shear component. Consider ``decompose()`` method to extract scale from transformations having non-trivial shear component.
    /// - Note: If determinant of the matrix is negative then X scale element is negative and other elements are non-negative.
    @inlinable public var scale: Vector { KvTransform3.scale(from: matrix) }

    /// Translation component of the receiver.
    ///
    /// - Warning: Assuming bottom row of the receiver's matrix is `[ 0, 0, 0, 1 ]`.
    @inlinable public var translation: Vector { KvTransform3.translation(from: matrix) }


    /// - Returns: Transformed basis vector for given index.
    ///
    /// See: ``basisX``, ``basisY``, ``basisZ``, ``Basis/vector(at:from:)``.
    @inlinable public func basisVector(at index: Int) -> Vector { Basis.vector(at: index, from: matrix) }


    /// - Returns: Tranformed normal.
    @inlinable public func act(normal n: Vector) -> Vector { normalMatrix * n }

    /// - Returns: Transformed coordinate.
    @inlinable public func act(coordinate c: Vector) -> Vector { KvTransform3.act(matrix, coordinate: c) }

    /// - Returns: Transformed vector.
    @inlinable public func act(vector v: Vector) -> Vector { KvTransform3.act(matrix, vector: v) }


    /// Fast implementation for product of the tranlation and the receiver.
    @inlinable
    public func translated(by translation: Vector) -> Self {
        var m = matrix
        var m⁻¹ = inverseMatrix
        let t4 = Math.make4(translation)

        m[3] += t4
        // - Note: Assuming bottom row of m⁻¹ is [ 0, 0, 1 ]
        m⁻¹[3] -= m⁻¹ * t4

        return Self(m, m⁻¹, normalMatrix)
    }



    // MARK: Operators

    @inlinable
    public static func *(lhs: Self, rhs: Self) -> Self {
        Self(lhs.matrix * rhs.matrix, inverseMatrix: rhs.inverseMatrix * lhs.inverseMatrix)
    }

    @inlinable
    public static func *(lhs: Self, rhs: KvAffineTransform3<Math>) -> Self {
        Self(lhs.matrix * Math.make4(rhs.matrix), inverseMatrix: Math.make4(rhs.inverseMatrix) * lhs.inverseMatrix)
    }

    @inlinable
    public static func *(lhs: KvAffineTransform3<Math>, rhs: Self) -> Self {
        Self(Math.make4(lhs.matrix) * rhs.matrix, inverseMatrix: rhs.inverseMatrix * Math.make4(lhs.inverseMatrix))
    }


    @inlinable
    public static func *(lhs: Self, rhs: Vector) -> Vector { lhs.act(vector: rhs) }

}


// MARK: : KvNumericallyEquatable

extension KvTransform3 : KvNumericallyEquatable {

    /// - Returns: A boolean value indicating whether the receiver and *rhs* are numerically equal.
    @inlinable public func isEqual(to rhs: Self) -> Bool { Math.isEqual(matrix, rhs.matrix) }

}


// MARK: : Equatable

extension KvTransform3 : Equatable {

    @inlinable public static func ==(lhs: Self, rhs: Self) -> Bool { lhs.matrix == rhs.matrix }

    @inlinable public static func !=(lhs: Self, rhs: Self) -> Bool { lhs.matrix != rhs.matrix }

}



// MARK: .Basis

extension KvTransform3 {

    /// Implementation of basis in 3D coordinate space with arbitrary origin.
    public struct Basis {

        /// Basis vector.
        public var x, y, z: Vector
        /// Translation.
        public var origin: Vector



        /// Memberwise initializer.
        @inlinable
        public init(x: Vector, y: Vector, z: Vector, origin: Vector = .zero) {
            self.x = x
            self.y = y
            self.z = z
            self.origin = origin
        }


        /// Initializes trivial basis.
        @inlinable public init() { self.init(x: .unitX, y: .unitY, z: .unitZ) }


        /// Initializes basis with vectors in given order.
        @inlinable
        public init(vectors: (Vector, Vector, Vector), in order: Permutation, origin: Vector) {
            switch order {
            case .xyz:
                (x, y, z) = vectors
            case .xzy:
                (x, z, y) = vectors
            case .yxz:
                (y, x, z) = vectors
            case .yzx:
                (y, z, x) = vectors
            case .zxy:
                (z, x, y) = vectors
            case .zyx:
                (z, y, x) = vectors
            }

            self.origin = origin
        }


        /// Initializes basis with vectors of given basis and origin.
        @inlinable
        public init(_ basis: KvAffineTransform3<Math>.Basis, origin: Vector = .zero) {
            self.init(x: basis.x, y: basis.y, z: basis.z, origin: origin)
        }


        /// Extracts basis from given transformation matrix.
        @inlinable
        public init(_ m: Matrix) {
            self.init(x: Basis.x(from: m), y: Basis.y(from: m), z: Basis.z(from: m), origin: KvTransform3.translation(from: m))
        }

        /// Extracts basis from given affine transformation matrix.
        @inlinable
        public init(_ m: KvAffineTransform3<Math>.Matrix) {
            typealias Basis = KvAffineTransform3<Math>.Basis

            self.init(x: Basis.x(from: m), y: Basis.y(from: m), z: Basis.z(from: m))
        }


        /// Extracts basis from given transformation.
        @inlinable public init(_ t: KvTransform3) { self.init(t.matrix) }

        /// Extracts basis from given affine transformation.
        @inlinable public init(_ t: KvAffineTransform3<Math>) { self.init(t.matrix) }



        // MARK: Subscripts

        /// Provides access to the receiver's vectors in given *order*.
        @inlinable
        public subscript(order: Permutation) -> (Vector, Vector, Vector) {
            get {
                switch order {
                case .xyz:
                    return (x, y, z)
                case .xzy:
                    return (x, z, y)
                case .yxz:
                    return (y, x, z)
                case .yzx:
                    return (y, z, x)
                case .zxy:
                    return (z, x, y)
                case .zyx:
                    return (z, y, x)
                }
            }
            set {
                switch order {
                case .xyz:
                    (x, y, z) = newValue
                case .xzy:
                    (x, z, y) = newValue
                case .yxz:
                    (y, x, z) = newValue
                case .yzx:
                    (y, z, x) = newValue
                case .zxy:
                    (z, x, y) = newValue
                case .zyx:
                    (z, y, x) = newValue
                }
            }
        }



        // MARK: Completion

        /// - Returns: A left-handed basis where z = *x* × *y*.
        @inlinable
        public static func completeLH(x: Vector, y: Vector, origin: Vector = .zero) -> Basis {
            Basis(x: x, y: y, z: Math.cross(x, y), origin: origin)
        }


        /// - Returns: A left-handed basis where y = *z* × *x*.
        @inlinable
        public static func completeLH(x: Vector, z: Vector, origin: Vector = .zero) -> Basis {
            Basis(x: x, y: Math.cross(z, x), z: z, origin: origin)
        }


        /// - Returns: A left-handed basis where x = *y* × *z*.
        @inlinable
        public static func completeLH(y: Vector, z: Vector, origin: Vector = .zero) -> Basis {
            Basis(x: Math.cross(y, z), y: y, z: z, origin: origin)
        }



        // MARK: Access

        /// Trivial basis.
        @inlinable public static var identity: Basis { Basis() }


        /// - Returns: Transformed basis vector for given *index*.
        ///
        /// See: ``x(from:)``, ``y(from:)``, ``z(from:)``, ``setVector(_:at:in:)``.
        @inlinable
        public static func vector(at index: Int, from m: Matrix) -> Vector {
            assert((0..<3).contains(index), "Invalid basis vector index (\(index)) for KvTransform3")
            return Math.make3(m[index])
        }


        /// Replaces basis vector at given *index* in given *matrix* with given *vector*.
        ///
        /// See: ``setX(in:)``, ``setY(in:)``, ``setZ(in:)``, ``vector(at:from:)``.
        @inlinable
        public static func setVector(_ v: Vector, at index: Int, in m: inout Matrix) {
            assert((0..<3).contains(index), "Invalid basis vector index (\(index)) for KvTransform3")
            m[index] = Math.make4(v)
        }


        /// - Returns: Transformed X basis vector.
        ///
        /// See: ``y(from:)``, ``z(from:)``, ``setX(_:in:)``, ``vector(at:from:)``.
        @inlinable public static func x(from m: Matrix) -> Vector { Math.make3(m[0]) }


        /// - Returns: Transformed Y basis vector.
        ///
        /// See: ``x(from:)``, ``z(from:)``, ``setY(_:in:)``, ``vector(at:from:)``.
        @inlinable public static func y(from m: Matrix) -> Vector { Math.make3(m[1]) }


        /// - Returns: Transformed Z basis vector.
        ///
        /// See: ``x(from:)``, ``y(from:)``, ``setZ(_:in:)``, ``vector(at:from:)``.
        @inlinable public static func z(from m: Matrix) -> Vector { Math.make3(m[2]) }


        /// Replaces X basis vector in given *matrix* with given value.
        ///
        /// See: ``setY(_:in:)``, ``setZ(_:in:)``, ``x(from:)``, ``setVector(_:at:in:)``.
        @inlinable public static func setX(_ v: Vector, in m: inout Matrix) { m[0] = Math.make4(v) }


        /// Replaces Y basis vector in given *matrix* with given value.
        ///
        /// See: ``setX(_:in:)``, ``setZ(_:in:)``, ``y(from:)``, ``setVector(_:at:in:)``.
        @inlinable public static func setY(_ v: Vector, in m: inout Matrix) { m[1] = Math.make4(v) }


        /// Replaces Z basis vector in given *matrix* with given value.
        ///
        /// See: ``setX(_:in:)``, ``setY(_:in:)``, ``z(from:)``, ``setVector(_:at:in:)``.
        @inlinable public static func setZ(_ v: Vector, in m: inout Matrix) { m[2] = Math.make4(v) }



        // MARK: Orthogonalization

        /// - Returns: The result of [modified Gram–Schmidt process](https://en.wikipedia.org/wiki/Gram–Schmidt_process).
        @inlinable
        public static func orthogonalized(vectors v: (Vector, Vector, Vector)) -> (Vector, Vector, Vector) {
            let v0 = v.0
            let l0⁻² = (1.0 as Scalar) / Math.length²(v0)

            let v1 = v.1 - v0 * (Math.dot(v0, v.1) * l0⁻²)
            var v2 = v.2 - v0 * (Math.dot(v0, v.2) * l0⁻²)

            v2 = v2 - v1 * (Math.dot(v1, v2) / Math.length²(v1))

            return (v0, v1, v2)
        }


        /// - Returns: The result of [modified Gram–Schmidt process](https://en.wikipedia.org/wiki/Gram–Schmidt_process).
        @inlinable
        public static func safeOrthogonalized(vectors v: (Vector, Vector, Vector)) -> (Vector, Vector, Vector)? {
            let v0 = v.0
            let l0² = Math.length²(v.0)

            guard KvIsNonzero(l0²) else { return nil }

            let l0⁻² = (1.0 as Scalar) / l0²

            let v1 = v.1 - v0 * (Math.dot(v0, v.1) * l0⁻²)
            let l1² = Math.length²(v1)

            guard KvIsNonzero(l1²) else { return nil }

            var v2 = v.2 - v0 * (Math.dot(v0, v.2) * l0⁻²)

            v2 = v2 - v1 * (Math.dot(v1, v2) / l1²)

            guard Math.isNonzero(v2) else { return nil }

            return (v0, v1, v2)
        }


        /// - Returns: The result of [modified Gram–Schmidt process](https://en.wikipedia.org/wiki/Gram–Schmidt_process) where each vector is normalized.
        @inlinable
        public static func orthonormalized(vectors v: (Vector, Vector, Vector)) -> (Vector, Vector, Vector) {
            let u0 = Math.normalize(v.0)
            let u1 = Math.normalize(v.1 - u0 * (Math.dot(u0, v.1)))

            let v2 = v.2 - u0 * Math.dot(u0, v.2)
            let u2 = Math.normalize(v2 - u1 * Math.dot(u1, v2))

            return (u0, u1, u2)
        }


        /// - Returns: The result of [modified Gram–Schmidt process](https://en.wikipedia.org/wiki/Gram–Schmidt_process) where each vector is normalized.
        ///            `Nil` is returned if any of vectors becomes degenerate.
        @inlinable
        public static func safeOrthonormalized(vectors v: (Vector, Vector, Vector)) -> (Vector, Vector, Vector)? {
            guard let u0 = Math.safeNormalize(v.0),
                  let u1 = Math.safeNormalize(v.1 - u0 * (Math.dot(u0, v.1)))
            else { return nil }

            let v2 = v.2 - u0 * Math.dot(u0, v.2)

            guard let u2 = Math.safeNormalize(v2 - u1 * Math.dot(u1, v2)) else { return nil }

            return (u0, u1, u2)
        }


        /// Applies [modified Gram–Schmidt process](https://en.wikipedia.org/wiki/Gram–Schmidt_process) in given *order* to the receiver.
        ///
        /// See: ``orthogonalized(order:)``.
        @inlinable
        public mutating func orthogonalize(order: Permutation = .xyz) {
            _ = { $0 = Basis.orthogonalized(vectors: $0) }(&self[order])
        }


        /// - Returns: The result of [modified Gram–Schmidt process](https://en.wikipedia.org/wiki/Gram–Schmidt_process) applied to the receiver's vectors in given *order*.
        ///
        /// See: ``orthogonalize(order:)``, ``safeOrthogonalized(order:)``.
        @inlinable
        public func orthogonalized(order: Permutation = .xyz) -> Basis {
            Basis(vectors: Basis.orthogonalized(vectors: self[order]), in: order, origin: origin)
        }


        /// - Returns: The result of [modified Gram–Schmidt process](https://en.wikipedia.org/wiki/Gram–Schmidt_process) applied to the receiver's vectors in given *order*.
        ///            `Nil` is returned when zero vector occurs.
        ///
        /// See: ``orthogonalized(order:)``.
        @inlinable
        public func safeOrthogonalized(order: Permutation = .xyz) -> Basis? {
            Basis.safeOrthogonalized(vectors: self[order])
                .map { Basis(vectors: $0, in: order, origin: origin) }
        }


        /// Applies [modified Gram–Schmidt process](https://en.wikipedia.org/wiki/Gram–Schmidt_process) in given *order* and normalizes to the receiver.
        ///
        /// See: ``orthonormalized(order:)``.
        @inlinable
        public mutating func orthonormalize(order: Permutation = .xyz) {
            _ = { $0 = Basis.orthonormalized(vectors: $0) }(&self[order])
        }


        /// - Returns: The normalized result of [modified Gram–Schmidt process](https://en.wikipedia.org/wiki/Gram–Schmidt_process) applied to the receiver's vectors in given *order*.
        ///
        /// See: ``orthonormalize(order:)``, ``safeOrthonormalized(order:)``.
        @inlinable
        public func orthonormalized(order: Permutation = .xyz) -> Basis {
            Basis(vectors: Basis.orthonormalized(vectors: self[order]), in: order, origin: origin)
        }


        /// - Returns: The normalized result of [modified Gram–Schmidt process](https://en.wikipedia.org/wiki/Gram–Schmidt_process) applied to the receiver's vectors in given *order*.
        ///            `Nil` is returned when zero vector occurs.
        ///
        /// See: ``orthonormalized(order:)``.
        @inlinable
        public func safeOrthonormalized(order: Permutation = .xyz) -> Basis? {
            Basis.safeOrthonormalized(vectors: self[order])
                .map { Basis(vectors: $0, in: order, origin: origin) }
        }



        // MARK: Operations

        /// A boolean value indicating whether the receiver is degenerate.
        @inlinable
        public var isDegenerate: Bool {
            let m = Math.Matrix3x3(x, y, z)
            return KvIsZero(m.determinant, eps: Math.epsArg(m).tolerance)
        }

        /// A boolean value indicating whether all the receiver's vectors are of unit length.
        @inlinable public var isNormalized: Bool { Math.isUnit(x) && Math.isUnit(y) && Math.isUnit(z) }

        /// A boolean value indicating wheter the receiiver is orthogonal.
        @inlinable public var isOrthogonal: Bool { Math.isOrthogonal(x, y) && Math.isOrthogonal(x, z) && Math.isOrthogonal(y, z) }


        /// Matrix representation of the receiver.
        ///
        /// See: ``transform``.
        @inlinable public var matrix: Matrix { Matrix(Math.make4(x), Math.make4(y), Math.make4(z), Matrix.Column(origin, 1.0 as Scalar)) }

        /// *KvTransform2* representation of the receiver.
        ///
        /// See: ``matrix``.
        @inlinable public var transform: KvTransform3 { KvTransform3(matrix) }


        /// Normalizes the receiver's vectors.
        ///
        /// See: ``normalized()``.
        @inlinable
        public mutating func normalize() {
            x = Math.normalize(x)
            y = Math.normalize(y)
            z = Math.normalize(z)
        }


        /// - Returns: A copy of the receiver where the vectors are normalized.
        ///
        /// See: ``normalize()``, ``safeNormalized()``.
        @inlinable public func normalized() -> Basis { Basis(x: Math.normalize(x), y: Math.normalize(y), z: Math.normalize(z), origin: origin) }


        /// - Returns: A copy of the receiver where all the vectors are normalized if nonzero. If any vector is zero then `nil` is returned.
        ///
        /// See: ``normalized()``.
        @inlinable
        public func safeNormalized() -> Basis? {
            guard let x = Math.safeNormalize(x),
                  let y = Math.safeNormalize(y),
                  let z = Math.safeNormalize(z)
            else { return nil }

            return Basis(x: x, y: y, z: z, origin: origin)
        }



        // MARK: .Permutation

        /// Permutation of 3D basis vectors.
        public enum Permutation { case xyz, xzy, yxz, yzx, zxy, zyx }

    }

}
