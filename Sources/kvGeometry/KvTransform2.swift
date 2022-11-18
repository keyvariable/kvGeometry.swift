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
//  KvTransform2.swift
//  kvGeometry
//
//  Created by Svyatoslav Popov on 18.11.2022.
//

import kvKit



/// Transformation of vectors in 2D coordinate space.
public struct KvTransform2<Math : KvMathScope> {

    public typealias Math = Math

    public typealias Scalar = Math.Scalar
    public typealias Matrix = Math.Matrix3x3
    public typealias NormalMatrix = Math.Matrix2x2

    public typealias Vector = Math.Vector2



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
        self.init(matrix,
                  inverseMatrix,
                  KvAffineTransform2<Math>.normalizedScaleComponent(for: Math.make2(inverseMatrix).transpose))
    }

    @usableFromInline
    internal init(_ matrix: Matrix, _ inverseMatrix: Matrix, _ normalMatrix: NormalMatrix) {
        self.matrix = matrix
        self.inverseMatrix = inverseMatrix
        self.normalMatrix = normalMatrix
    }


    /// See ``init(_:translation:)``, ``init(_:relativeTo:)``.
    @inlinable
    public init(_ t: KvAffineTransform2<Math>) {
        self.init(Math.make3(t.matrix), Math.make3(t.inverseMatrix), t.normalMatrix)
    }


    /// Initializes combination of given affine tranformation and translation.
    ///
    /// See ``init(_:)``, ``init(_:relativeTo:)``.
    @inlinable
    public init(_ t: KvAffineTransform2<Math>, translation: Vector) {
        self.init(KvTransform2.makeMatrix(t.matrix, translation: translation),
                  KvTransform2.makeMatrix(t.inverseMatrix, translation: t.inverseMatrix * -translation),
                  t.normalMatrix)
    }


    /// Initializes transformation equal to application of given affine tranformation relative to given coordinate.
    /// It equal to T(by: *translation*) × *t* × T(by: –*translation*).
    ///
    /// E.g. rotation relative to a coordinate may be initialized this way.
    ///
    /// See ``init(_:)``, ``init(_:translation:)``.
    @inlinable
    public init(_ t: KvAffineTransform2<Math>, relativeTo coordinate: Vector) {
        self.init(t, translation: coordinate - t.matrix * coordinate)
    }


    /// Initializes a rotation transformation.
    @inlinable
    public init(angle: Scalar) {
        let (sine, cosine) = Math.sincos(angle)

        self.init(
            Matrix(Matrix.Column( cosine, sine, 0),
                   Matrix.Column(-sine, cosine, 0),
                   Matrix.Column.unitZ),
            inverseMatrix: Matrix(Matrix.Column(cosine, -sine, 0),
                                  Matrix.Column( sine, cosine, 0),
                                  Matrix.Column.unitZ)
        )
    }


    /// Initializes product of rotation and scale transformations.
    @inlinable
    public init(angle: Scalar, scale: Vector) {
        let scale⁻¹ = Matrix.Column(1 / scale, 1)
        let (sine, cosine) = Math.sincos(angle)

        self.init(
            Matrix(Matrix.Column( cosine, sine, 0) * scale.x,
                   Matrix.Column(-sine, cosine, 0) * scale.y,
                   Matrix.Column.unitZ),
            inverseMatrix: Matrix(Matrix.Column(cosine, -sine, 0) * scale⁻¹,
                                  Matrix.Column( sine, cosine, 0) * scale⁻¹,
                                  Matrix.Column.unitZ)
        )
    }

    /// Initializes product of rotation and scale transformations.
    @inlinable public init(angle: Scalar, scale: Scalar) { self.init(angle: angle, scale: Vector(repeating: scale)) }


    /// Initializes a scale transformation.
    @inlinable
    public init(scale: Vector) {
        let scale⁻¹ = 1 / scale

        self.init(
            Matrix(diagonal: Matrix.Diagonal(scale, 1)),
            Matrix(diagonal: Matrix.Diagonal(scale⁻¹, 1)),
            NormalMatrix(diagonal: KvAffineTransform2<Math>.normalizedScaleComponent(for: scale⁻¹))
        )
    }

    /// Initializes a scale transformation.
    @inlinable public init(scale: Scalar) { self.init(scale: Vector(repeating: scale)) }


    /// Initializes a translation transformation.
    @inlinable
    public init(translation: Vector) {
        self.init(
            Matrix(Matrix.Column.unitX,
                   Matrix.Column.unitY,
                   Matrix.Column(translation, 1)),
            Matrix(Matrix.Column.unitX,
                   Matrix.Column.unitY,
                   Matrix.Column(-translation, 1)),
            .identity
        )
    }


    /// Initializes product of translation and rotation transformations.
    @inlinable
    public init(translation: Vector, angle: Scalar) {
        let r = NormalMatrix(angle: angle)
        let r⁻¹ = r.transpose
        let t⁻¹ = -translation

        self.init(
            Matrix(Math.make3(r[0]),
                   Math.make3(r[1]),
                   Matrix.Column(translation, 1)),
            Matrix(Math.make3(r⁻¹[0]),
                   Math.make3(r⁻¹[1]),
                   Matrix.Column(Math.dot(r[0], t⁻¹), Math.dot(r[1], t⁻¹), 1)),
            r
        )
    }


    /// Initializes product of translation, rotation and scale transformations.
    @inlinable
    public init(translation: Vector, angle: Scalar, scale: Vector) {
        var r = NormalMatrix(angle: angle)
        var r⁻¹ = r.transpose

        let scale⁻¹ = 1 / scale
        let t⁻¹ = (r⁻¹ * -translation) * scale⁻¹

        r[0] *= scale.x
        r[1] *= scale.y

        r⁻¹[0] *= scale⁻¹
        r⁻¹[1] *= scale⁻¹

        self.init(
            Matrix(Math.make3(r[0]),
                   Math.make3(r[1]),
                   Matrix.Column(translation, 1)),
            inverseMatrix: Matrix(Math.make3(r⁻¹[0]),
                                  Math.make3(r⁻¹[1]),
                                  Matrix.Column(t⁻¹, 1))
        )
    }

    /// Initializes product of translation, rotation and scale transformations.
    @inlinable
    public init(translation: Vector, angle: Scalar, scale: Scalar) {
        self.init(translation: translation, angle: angle, scale: Vector(repeating: scale))
    }


    /// Initializes product of translation and scale transformations.
    @inlinable
    public init(translation: Vector, scale: Vector) {
        let scale⁻¹ = 1 / scale

        self.init(
            Matrix(Matrix.Column(scale.x, 0, 0),
                   Matrix.Column(0, scale.y, 0),
                   Matrix.Column(translation, 1)),
            Matrix(Matrix.Column(scale⁻¹.x, 0, 0),
                   Matrix.Column(0, scale⁻¹.y, 0),
                   Matrix.Column(-translation * scale⁻¹, 1)),
            NormalMatrix(diagonal: KvAffineTransform2<Math>.normalizedScaleComponent(for: scale⁻¹))
        )
    }

    /// Initializes product of translation and scale transformations.
    @inlinable
    public init(translation: Vector, scale: Scalar) { self.init(translation: translation, scale: Vector(repeating: scale)) }



    // MARK: Auxiliaries

    @inlinable public static var identity: Self { .init() }


    /// - Returns: Scale component of given tranform matrix.
    ///
    /// - Warning: Assuming bottom row of the matrix is `[ 0, 0, 1 ]`.
    /// - Note: If determinant of the matrix is negative then X scale element is negative and other elements are non-negative.
    @inlinable
    public static func scale(from m: Matrix) -> Vector {
        Vector(x: Math.length(m[0]) * (KvIsNotNegative(m.determinant) ? 1 : -1),
               y: Math.length(m[1]))
    }


    /// Translation component of the receiver.
    ///
    /// - Warning: Assuming bottom row of the receiver's matrix is `[ 0, 0, 1 ]`.
    @inlinable
    public static func translation(from m: Matrix) -> Vector { Math.make2(m[2]) }


    /// Transformed X basis vector.
    ///
    /// See: ``basisY(from:)``, ``basisX``.
    @inlinable public static func basisX(from m: Matrix) -> Vector { Math.make2(m[0]) }

    /// Transformed Y basis vector.
    ///
    /// See: ``basisX(from:)``, ``basisY``.
    @inlinable public static func basisY(from m: Matrix) -> Vector { Math.make2(m[1]) }


    /// - Returns: Transformed coordinate by a transformation represented as given matrix.
    @inlinable
    public static func act(_ matrix: Matrix, coordinate c: Vector) -> Vector {
        let c3 = matrix * Matrix.Column(c, 1)
        return Math.make2(c3) / c3.z
    }


    /// - Returns: Transformed vector by a transformation represented as given matrix.
    @inlinable
    public static func act(_ matrix: Matrix, vector v: Vector) -> Vector {
        Math.make2(matrix * Math.make3(v))
    }


    /// - Returns: Combination of given affine tranformation matrix and translation.
    ///
    /// See ``makeMatrix(_:relativeTo:)``.
    @inlinable
    public static func makeMatrix(_ a: KvAffineTransform2<Math>.Matrix, translation: Vector) -> Matrix {
        Matrix(Math.make3(a[0]),
               Math.make3(a[1]),
               Matrix.Column(translation, 1))
    }


    /// - Returns: Transformation matrix representing application of given affine tranformation relative to given coordinate. It equal to T(by: *translation*) × *a* × T(by: –*translation*).
    ///
    /// E.g. rotation relative to a coordinate may be initialized this way.
    ///
    /// See ``makeMatrix(_:translation:)``.
    @inlinable
    public static func makeMatrix(_ a: KvAffineTransform2<Math>.Matrix, relativeTo coordinate: Vector) -> Matrix {
        makeMatrix(a, translation: coordinate - a * coordinate)
    }


    /// - Returns: Rotation transformation matrix.
    @inlinable
    public static func makeMatrix(angle: Scalar) -> Matrix {
        let (sine, cosine) = Math.sincos(angle)
        return Matrix(Matrix.Column(cosine, sine, 0),
                      Matrix.Column(-sine, cosine, 0),
                      .unitZ)
    }


    /// - Returns: Rotation-scale transformation matrix.
    @inlinable
    public static func makeMatrix(angle: Scalar, scale: Vector) -> Matrix {
        let (sine, cosine) = Math.sincos(angle)
        return Matrix(Matrix.Column(cosine, sine, 0) * scale.x,
                      Matrix.Column(-sine, cosine, 0) * scale.y,
                      .unitZ)
    }

    /// - Returns: Rotation-scale transformation matrix.
    @inlinable public static func makeMatrix(angle: Scalar, scale: Scalar) -> Matrix { makeMatrix(angle: angle, scale: Vector(repeating: scale)) }


    /// - Returns: Scale transformation matrix.
    @inlinable public static func makeMatrix(scale: Vector) -> Matrix { Matrix(diagonal: Matrix.Diagonal(scale, 1)) }

    /// - Returns: Scale transformation matrix.
    @inlinable public static func makeMatrix(scale: Scalar) -> Matrix { Matrix(diagonal: Matrix.Diagonal(scale, scale, 1)) }


    /// - Returns: Translation transformation matrix.
    @inlinable public static func makeMatrix(translation: Vector) -> Matrix { Matrix(.unitX, .unitY, Matrix.Column(translation, 1)) }


    /// - Returns: Translation-rotation transformation matrix.
    @inlinable
    public static func makeMatrix(translation: Vector, angle: Scalar) -> Matrix {
        let (sine, cosine) = Math.sincos(angle)
        return Matrix(Matrix.Column(cosine, sine, 0),
                      Matrix.Column(-sine, cosine, 0),
                      Matrix.Column(translation, 1))
    }


    /// - Returns: Translation-rotation-scale transformation matrix.
    @inlinable
    public static func makeMatrix(translation: Vector, angle: Scalar, scale: Vector) -> Matrix {
        let (sine, cosine) = Math.sincos(angle)
        return Matrix(Matrix.Column(cosine, sine, 0) * scale.x,
                      Matrix.Column(-sine, cosine, 0) * scale.y,
                      Matrix.Column(translation, 1))
    }

    /// - Returns: Translation-rotation-scale transformation matrix.
    @inlinable
    public static func makeMatrix(translation: Vector, angle: Scalar, scale: Scalar) -> Matrix {
        makeMatrix(translation: translation, angle: angle, scale: Vector(repeating: scale))
    }


    /// - Returns: Translation-scale transformation matrix.
    @inlinable
    public static func makeMatrix(translation: Vector, scale: Vector) -> Matrix {
        Matrix(Matrix.Column(scale.x, 0, 0),
               Matrix.Column(0, scale.y, 0),
               Matrix.Column(translation, 1))
    }

    /// - Returns: Translation-scale transformation matrix.
    @inlinable public static func makeMatrix(translation: Vector, scale: Scalar) -> Matrix { makeMatrix(scale: Vector(repeating: scale)) }



    // MARK: Operations

    /// Transformed X basis vector.
    ///
    /// See: ``basisY``, ``basisX(from:)``.
    @inlinable public var basisX: Vector { KvTransform2.basisX(from: matrix) }
    /// Transformed Y basis vector.
    ///
    /// See: ``basisX``, ``basisY(from:)``.
    @inlinable public var basisY: Vector { KvTransform2.basisY(from: matrix) }


    /// A boolean value indicating whether the receiver is numerically equal to identity tranformation.
    @inlinable public var isIdentity: Bool { Math.isEqual(matrix, .identity) }


    /// The inverse transform.
    @inlinable
    public var inverse: Self {
        Self(inverseMatrix, matrix, KvAffineTransform2<Math>.normalizedScaleComponent(for: Math.make2(matrix).transpose))
    }


    /// Scale component of the receiver.
    ///
    /// - Warning: Assuming bottom row of the receiver's matrix is `[ 0, 0, 1 ]`.
    /// - Note: If determinant of the matrix is negative then X scale element is negative and other elements are non-negative.
    @inlinable public var scale: Vector { KvTransform2.scale(from: matrix) }

    /// Translation component of the receiver.
    ///
    /// - Warning: Assuming bottom row of the receiver's matrix is `[ 0, 0, 1 ]`.
    @inlinable public var translation: Vector { KvTransform2.translation(from: matrix) }


    /// - Returns: Tranformed normal.
    @inlinable public func act(normal n: Vector) -> Vector { normalMatrix * n }

    /// - Returns: Transformed coordinate.
    @inlinable public func act(coordinate c: Vector) -> Vector { KvTransform2.act(matrix, coordinate: c) }

    /// - Returns: Transformed vector.
    @inlinable public func act(vector v: Vector) -> Vector { KvTransform2.act(matrix, vector: v) }


    /// Fast implementation for product of the tranlation and the receiver.
    @inlinable
    public func translated(by translation: Vector) -> Self {
        var m = matrix
        var m⁻¹ = inverseMatrix
        let t3 = Math.make3(translation)

        m[2] += t3
        // - Note: Assuming bottom row of m⁻¹ is [ 0, 0, 1 ]
        m⁻¹[2] -= m⁻¹ * t3

        return Self(m, m⁻¹, normalMatrix)
    }



    // MARK: Operators

    @inlinable
    public static func *(lhs: Self, rhs: Self) -> Self {
        Self(lhs.matrix * rhs.matrix, inverseMatrix: rhs.inverseMatrix * lhs.inverseMatrix)
    }

    @inlinable
    public static func *(lhs: Self, rhs: KvAffineTransform2<Math>) -> Self {
        Self(lhs.matrix * Math.make3(rhs.matrix), inverseMatrix: Math.make3(rhs.inverseMatrix) * lhs.inverseMatrix)
    }

    @inlinable
    public static func *(lhs: KvAffineTransform2<Math>, rhs: Self) -> Self {
        Self(Math.make3(lhs.matrix) * rhs.matrix, inverseMatrix: rhs.inverseMatrix * Math.make3(lhs.inverseMatrix))
    }


    @inlinable
    public static func *(lhs: Self, rhs: Vector) -> Vector { lhs.act(vector: rhs) }

}


// MARK: : KvNumericallyEquatable

extension KvTransform2 : KvNumericallyEquatable {

    /// - Returns: A boolean value indicating whether the receiver and *rhs* are numerically equal.
    @inlinable public func isEqual(to rhs: Self) -> Bool { Math.isEqual(matrix, rhs.matrix) }

}


// MARK: : Equatable

extension KvTransform2 : Equatable {

    @inlinable public static func ==(lhs: Self, rhs: Self) -> Bool { lhs.matrix == rhs.matrix }

    @inlinable public static func !=(lhs: Self, rhs: Self) -> Bool { lhs.matrix != rhs.matrix }

}
