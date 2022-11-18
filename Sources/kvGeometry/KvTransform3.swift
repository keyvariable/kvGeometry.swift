//===----------------------------------------------------------------------===//
//
//  Copyright (c) 2021 Svyatoslav Popov.
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
        let scale⁻¹ = Matrix.Column(1 / scale, 1)

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
        let scale⁻¹ = 1 / scale

        self.init(
            Matrix(diagonal: Matrix.Diagonal(scale, 1)),
            Matrix(diagonal: Matrix.Diagonal(scale⁻¹, 1)),
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
                   Matrix.Column(translation, 1)),
            Matrix(.unitX,
                   .unitY,
                   .unitZ,
                   Matrix.Column(-translation, 1)),
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
                   Matrix.Column(translation, 1)),
            Matrix(Math.make4(r⁻¹[0]),
                   Math.make4(r⁻¹[1]),
                   Math.make4(r⁻¹[2]),
                   Matrix.Column(Math.dot(r[0], t⁻¹), Math.dot(r[1], t⁻¹), Math.dot(r[2], t⁻¹), 1)),
            r
        )
    }


    /// Initializes product of translation, rotation and scale transformations.
    @inlinable
    public init(translation: Vector, quaternion: Math.Quaternion, scale: Vector) {
        var r = NormalMatrix(quaternion)
        var r⁻¹ = r.transpose

        let scale⁻¹ = 1 / scale
        let t⁻¹ = (r⁻¹ * -translation) * scale⁻¹

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
                   Matrix.Column(translation, 1)),
            inverseMatrix: Matrix(Math.make4(r⁻¹[0]),
                                  Math.make4(r⁻¹[1]),
                                  Math.make4(r⁻¹[2]),
                                  Matrix.Column(t⁻¹, 1))
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
        let scale⁻¹ = 1 / scale

        self.init(
            Matrix(Matrix.Column(scale.x, 0, 0, 0),
                   Matrix.Column(0, scale.y, 0, 0),
                   Matrix.Column(0, 0, scale.z, 0),
                   Matrix.Column(translation, 1)),
            Matrix(Matrix.Column(scale⁻¹.x, 0, 0, 0),
                   Matrix.Column(0, scale⁻¹.y, 0, 0),
                   Matrix.Column(0, 0, scale⁻¹.z, 0),
                   Matrix.Column(-translation * scale⁻¹, 1)),
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
    /// - Note: If determinant of the matrix is negative then X scale element is negative and other elements are non-negative.
    @inlinable
    public static func scale(from m: Matrix) -> Vector {
        Vector(x: Math.length(m[0]) * (KvIsNotNegative(m.determinant) ? 1 : -1),
               y: Math.length(m[1]),
               z: Math.length(m[2]))
    }


    /// Translation component of the receiver.
    ///
    /// - Warning: Assuming bottom row of the receiver's matrix is `[ 0, 0, 0, 1 ]`.
    @inlinable
    public static func translation(from m: Matrix) -> Vector { Math.make3(m[3]) }


    /// Transformed basis vector for given index.
    ///
    /// See: ``basisX(from:)``, ``basisY(from:)``, ``basisZ(from:)``.
    @inlinable
    public static func basis(_ index: Int, from m: Matrix) -> Vector {
        assert((0..<3).contains(index), "Unvalid basis vector index (\(index)) for KvTransform3")
        return Math.make3(m[index])
    }


    /// Transformed X basis vector.
    ///
    /// See: ``basisY(from:)``, ``basisZ(from:)``, ``basisX``, ``basis(_:from:)``.
    @inlinable public static func basisX(from m: Matrix) -> Vector { Math.make3(m[0]) }

    /// Transformed Y basis vector.
    ///
    /// See: ``basisX(from:)``, ``basisZ(from:)``, ``basisY``, ``basis(_:from:)``.
    @inlinable public static func basisY(from m: Matrix) -> Vector { Math.make3(m[1]) }

    /// Transformed Z basis vector.
    ///
    /// See: ``basisX(from:)``, ``basisY(from:)``, ``basisZ``, ``basis(_:from:)``.
    @inlinable public static func basisZ(from m: Matrix) -> Vector { Math.make3(m[2]) }


    /// - Returns: Transformed coordinate by a transformation represented as given matrix.
    @inlinable
    public static func act(_ matrix: Matrix, coordinate c: Vector) -> Vector {
        let c4 = matrix * Matrix.Column(c, 1)
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
               Matrix.Column(translation, 1))
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
    @inlinable public static func makeMatrix(scale: Vector) -> Matrix { Matrix(diagonal: Matrix.Diagonal(scale, 1)) }

    /// - Returns: Scale transformation matrix.
    @inlinable public static func makeMatrix(scale: Scalar) -> Matrix { Matrix(diagonal: Matrix.Diagonal(scale, scale, scale, 1)) }


    /// - Returns: Translation transformation matrix.
    @inlinable public static func makeMatrix(translation: Vector) -> Matrix { Matrix(.unitX, .unitY, .unitZ, Matrix.Column(translation, 1)) }


    /// - Returns: Translation-rotation transformation matrix.
    @inlinable
    public static func makeMatrix(translation: Vector, quaternion: Math.Quaternion) -> Matrix {
        var m = Matrix(quaternion)
        m[3] = Matrix.Column(translation, 1)
        return m
    }


    /// - Returns: Translation-rotation-scale transformation matrix.
    @inlinable
    public static func makeMatrix(translation: Vector, quaternion: Math.Quaternion, scale: Vector) -> Matrix {
        var m = Matrix(quaternion)
        m[0] *= scale.x
        m[1] *= scale.y
        m[2] *= scale.z
        m[3] = Matrix.Column(translation, 1)
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
        Matrix(Matrix.Column(scale.x, 0, 0, 0),
               Matrix.Column(0, scale.y, 0, 0),
               Matrix.Column(0, 0, scale.z, 0),
               Matrix.Column(translation, 1))
    }

    /// - Returns: Translation-scale transformation matrix.
    @inlinable public static func makeMatrix(translation: Vector, scale: Scalar) -> Matrix { makeMatrix(scale: Vector(repeating: scale)) }



    // MARK: Projections

    /// - Returns: Matrix of standard orthogonal projection.
    @inlinable
    public static func orthogonalProjectionMatrix(left: Scalar, right: Scalar, top: Scalar, bottom: Scalar, near: Scalar, far: Scalar) -> Matrix {
        Matrix(diagonal: 1 / Matrix.Diagonal(left - right, bottom - top, near - far, 1))
        * Matrix([ -2,  0, 0, 0 ],
                 [  0, -2, 0, 0 ],
                 [  0,  0, 2, 0 ],
                 Matrix.Column(right + left, top + bottom, far + near, 1))
    }


    /// - Parameter aspect: Ratio of Viewport width to viewport height.
    /// - Parameter fof: Vertical camera angle.
    ///
    /// - Returns: Projection matrix for a centered rectangular pinhole camera.
    @inlinable
    public static func perspectiveProjectionMatrix(aspect: Scalar, fov: Scalar, near: Scalar, far: Scalar) -> Matrix {
        let tg = Math.tan(0.5 * fov)

        return (Matrix(diagonal: 1 / Matrix.Diagonal(aspect * tg, tg, near - far, 1))
                * Matrix(Matrix.Column.unitX,
                         Matrix.Column.unitY,
                         Matrix.Column(0,  0,  (far + near)  , -1),
                         Matrix.Column(0,  0,  2 * far * near,  0)))
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
        * Matrix(Matrix.Column(k[0], 0),
                 Matrix.Column(k[1], 0),
                 Matrix.Column(-k[2], near + far),
                 Matrix.Column(0, 0, 0, near * far))
    }



    // MARK: Operations

    /// Transformed X basis vector.
    ///
    /// See: ``basisY``, ``basisZ``, ``basisX(from:)``, ``basis(_:)``.
    @inlinable public var basisX: Vector { KvTransform3.basisX(from: matrix) }
    /// Transformed Y basis vector.
    ///
    /// See: ``basisX``, ``basisZ``, ``basisY(from:)``, ``basis(_:)``.
    @inlinable public var basisY: Vector { KvTransform3.basisY(from: matrix) }
    /// Transformed Z basis vector.
    ///
    /// See: ``basisX``, ``basisY``, ``basisZ(from:)``, ``basis(_:)``.
    @inlinable public var basisZ: Vector { KvTransform3.basisZ(from: matrix) }


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
    /// - Note: If determinant of the matrix is negative then X scale element is negative and other elements are non-negative.
    @inlinable public var scale: Vector { KvTransform3.scale(from: matrix) }

    /// Translation component of the receiver.
    ///
    /// - Warning: Assuming bottom row of the receiver's matrix is `[ 0, 0, 0, 1 ]`.
    @inlinable public var translation: Vector { KvTransform3.translation(from: matrix) }


    /// - Returns: Transformed basis vector for given index.
    ///
    /// See: ``basisX``, ``basisY``, ``basisZ``, ``basis(_:from:)``.
    @inlinable public func basis(_ index: Int) -> Vector { KvTransform3.basis(index, from: matrix) }


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
