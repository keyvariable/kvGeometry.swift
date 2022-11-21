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
    /// See: ``basisY``, ``Basis/x(from:)``, ``basisVector(at:)``.
    @inlinable public var basisX: Vector { Basis.x(from: matrix) }
    /// Transformed Y basis vector.
    ///
    /// See: ``basisX``, ``Basis/y(from:)``, ``basisVector(at:)``.
    @inlinable public var basisY: Vector { Basis.y(from: matrix) }


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


    /// - Returns: Transformed basis vector for given index.
    ///
    /// See: ``basisX``, ``basisY``, ``Basis/vector(at:from:).
    @inlinable public func basisVector(at index: Int) -> Vector { Basis.vector(at: index, from: matrix) }


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



// MARK: .Basis

extension KvTransform2 {

    /// Implementation of basis in 2D coordinate space with arbitrary origin.
    public struct Basis {

        /// Basis vector.
        public var x, y: Vector
        /// Translation.
        public var origin: Vector



        /// Memberwise initializer.
        @inlinable
        public init(x: Vector, y: Vector, origin: Vector = .zero) {
            self.x = x
            self.y = y
            self.origin = origin
        }


        /// Initializes trivial basis.
        @inlinable public init() { self.init(x: .unitX, y: .unitY) }


        /// Initializes basis with vectors in given order.
        @inlinable
        public init(vectors: (Vector, Vector), in order: Permutation, origin: Vector) {
            switch order {
            case .xy:
                (x, y) = vectors
            case .yx:
                (y, x) = vectors
            }

            self.origin = origin
        }


        /// Initializes basis with vectors of given basis and origin.
        @inlinable
        public init(_ basis: KvAffineTransform2<Math>.Basis, origin: Vector = .zero) {
            self.init(x: basis.x, y: basis.y, origin: origin)
        }


        /// Extracts basis from given transformation matrix.
        @inlinable
        public init(_ m: Matrix) {
            self.init(x: Basis.x(from: m), y: Basis.y(from: m), origin: KvTransform2.translation(from: m))
        }

        /// Extracts basis from given affine transformation matrix.
        @inlinable
        public init(_ m: KvAffineTransform2<Math>.Matrix) {
            typealias Basis = KvAffineTransform2<Math>.Basis

            self.init(x: Basis.x(from: m), y: Basis.y(from: m))
        }


        /// Extracts basis from given transformation.
        @inlinable public init(_ t: KvTransform2) { self.init(t.matrix) }

        /// Extracts basis from given affine transformation.
        @inlinable public init(_ t: KvAffineTransform2<Math>) { self.init(t.matrix) }



        // MARK: Subscripts

        /// Provides access to the receiver's vectors in given *order*.
        @inlinable
        public subscript(order: Permutation) -> (Vector, Vector) {
            get {
                switch order {
                case .xy:
                    return (x, y)
                case .yx:
                    return (y, x)
                }
            }
            set {
                switch order {
                case .xy:
                    (x, y) = newValue
                case .yx:
                    (y, x) = newValue
                }
            }
        }



        // MARK: Completion

        /// - Returns: An orthogonal left-handed basis where Y vector is calculated for given X vector.
        ///
        /// - Note: Y vector has the same length as *x*.
        @inlinable public static func completeLH(x: Vector, origin: Vector = .zero) -> Basis { Basis(x: x, y: Vector(-x.y, x.x), origin: origin) }


        /// - Returns: An orthogonal left-handed basis where X vector is calculated for given Y vector.
        ///
        /// - Note: X vector has the same length as *y*.
        @inlinable public static func completeLH(y: Vector, origin: Vector = .zero) -> Basis { Basis(x: Vector(y.y, -y.x), y: y, origin: origin) }



        // MARK: Access

        /// Trivial basis.
        @inlinable public static var identity: Basis { Basis() }


        /// - Returns: Transformed basis vector for given *index*.
        ///
        /// See: ``x(from:)``, ``y(from:)``, ``setVector(_:at:in:)``.
        @inlinable
        public static func vector(at index: Int, from m: Matrix) -> Vector {
            assert((0..<2).contains(index), "Invalid basis vector index (\(index)) for KvTransform2")
            return Math.make2(m[index])
        }


        /// Replaces basis vector at given *index* in given *matrix* with given *vector*.
        ///
        /// See: ``setX(in:)``, ``setY(in:)``, ``vector(at:from:)``.
        @inlinable
        public static func setVector(_ v: Vector, at index: Int, in m: inout Matrix) {
            assert((0..<2).contains(index), "Invalid basis vector index (\(index)) for KvTransform2")
            m[index] = Math.make3(v)
        }


        /// - Returns: Transformed X basis vector.
        ///
        /// See: ``y(from:)``, ``setX(_:in:)``, ``vector(at:from:)``.
        @inlinable public static func x(from m: Matrix) -> Vector { Math.make2(m[0]) }


        /// - Returns: Transformed Y basis vector.
        ///
        /// See: ``x(from:)``, ``setY(_:in:)``, ``vector(at:from:)``.
        @inlinable public static func y(from m: Matrix) -> Vector { Math.make2(m[1]) }


        /// Replaces X basis vector in given *matrix* with given value.
        ///
        /// See: ``setY(_:in:)``, ``x(from:)``, ``setVector(_:at:in:)``.
        @inlinable public static func setX(_ v: Vector, in m: inout Matrix) { m[0] = Math.make3(v) }


        /// Replaces Y basis vector in given *matrix* with given value.
        ///
        /// See: ``setX(_:in:)``, ``y(from:)``, ``setVector(_:at:in:)``.
        @inlinable public static func setY(_ v: Vector, in m: inout Matrix) { m[1] = Math.make3(v) }



        // MARK: Orthogonalization

        /// - Returns: The result of [modified Gram–Schmidt process](https://en.wikipedia.org/wiki/Gram–Schmidt_process).
        @inlinable
        public static func orthogonalized(vectors v: (Vector, Vector)) -> (Vector, Vector) {
            (v.0, v.1 - v.0 * (Math.dot(v.0, v.1) / Math.length²(v.0)))
        }


        /// - Returns: The result of [modified Gram–Schmidt process](https://en.wikipedia.org/wiki/Gram–Schmidt_process).
        @inlinable
        public static func safeOrthogonalized(vectors v: (Vector, Vector)) -> (Vector, Vector)? {
            let l0² = Math.length²(v.0)

            guard KvIsNonzero(l0², eps: .zero²) else { return nil }

            return (v.0, v.1 - v.0 * (Math.dot(v.0, v.1) / l0²))
        }


        /// - Returns: The result of [modified Gram–Schmidt process](https://en.wikipedia.org/wiki/Gram–Schmidt_process) where each vector is normalized.
        @inlinable
        public static func orthonormalized(vectors v: (Vector, Vector)) -> (Vector, Vector) {
            let u0 = Math.normalize(v.0)
            let u1 = Math.normalize(v.1 - u0 * Math.dot(u0, v.1))

            return (u0, u1)
        }


        /// - Returns: The result of [modified Gram–Schmidt process](https://en.wikipedia.org/wiki/Gram–Schmidt_process) where each vector is normalized.
        ///            `Nil` is returned if any of vectors becomes degenerate.
        @inlinable
        public static func safeOrthonormalized(vectors v: (Vector, Vector)) -> (Vector, Vector)? {
            guard let u0 = Math.safeNormalize(v.0),
                  let u1 = Math.safeNormalize(v.1 - u0 * Math.dot(u0, v.1))
            else { return nil }

            return (u0, u1)
        }


        /// Applies [modified Gram–Schmidt process](https://en.wikipedia.org/wiki/Gram–Schmidt_process) in given *order* to the receiver.
        ///
        /// See: ``orthogonalized(order:)``.
        @inlinable
        public mutating func orthogonalize(order: Permutation = .xy) {
            _ = { Basis.orthogonalized(vectors: $0) }(&self[order])
        }


        /// - Returns: The result of [modified Gram–Schmidt process](https://en.wikipedia.org/wiki/Gram–Schmidt_process) applied to the receiver's vectors in given *order*.
        ///
        /// See: ``orthogonalize(order:)``, ``safeOrthogonalized(order:)``.
        @inlinable
        public func orthogonalized(order: Permutation = .xy) -> Basis {
            Basis(vectors: Basis.orthogonalized(vectors: self[order]), in: order, origin: origin)
        }


        /// - Returns: The result of [modified Gram–Schmidt process](https://en.wikipedia.org/wiki/Gram–Schmidt_process) applied to the receiver's vectors in given *order*.
        ///            `Nil` is returned when zero vector occurs.
        ///
        /// See: ``orthogonalized(order:)``.
        @inlinable
        public func safeOrthogonalized(order: Permutation = .xy) -> Basis? {
            Basis.safeOrthogonalized(vectors: self[order])
                .map { Basis(vectors: $0, in: order, origin: origin) }
        }


        /// Applies [modified Gram–Schmidt process](https://en.wikipedia.org/wiki/Gram–Schmidt_process) in given *order* and normalizes to the receiver.
        ///
        /// See: ``orthonormalized(order:)``.
        @inlinable
        public mutating func orthonormalize(order: Permutation = .xy) {
            _ = { Basis.orthonormalized(vectors: $0) }(&self[order])
        }


        /// - Returns: The normalized result of [modified Gram–Schmidt process](https://en.wikipedia.org/wiki/Gram–Schmidt_process) applied to the receiver's vectors in given *order*.
        ///
        /// See: ``orthonormalize(order:)``, ``safeOrthonormalized(order:)``.
        @inlinable
        public func orthonormalized(order: Permutation = .xy) -> Basis {
            Basis(vectors: Basis.orthonormalized(vectors: self[order]), in: order, origin: origin)
        }


        /// - Returns: The normalized result of [modified Gram–Schmidt process](https://en.wikipedia.org/wiki/Gram–Schmidt_process) applied to the receiver's vectors in given *order*.
        ///            `Nil` is returned when zero vector occurs.
        ///
        /// See: ``orthonormalized(order:)``.
        @inlinable
        public func safeOrthonormalized(order: Permutation = .xy) -> Basis? {
            Basis.safeOrthonormalized(vectors: self[order])
                .map { Basis(vectors: $0, in: order, origin: origin) }
        }



        // MARK: Operations

        /// A boolean value indicating whether the receiver is degenerate.
        @inlinable
        public var isDegenerate: Bool {
            let m = Math.Matrix2x2(x, y)
            return KvIsZero(m.determinant, eps: Math.epsArg(m).tolerance)
        }

        /// A boolean value indicating whether all the receiver's vectors are of unit length.
        @inlinable public var isNormalized: Bool { Math.isUnit(x) && Math.isUnit(y) }

        /// A boolean value indicating wheter the receiiver is orthogonal.
        @inlinable public var isOrthogonal: Bool { Math.isOrthogonal(x, y) }


        /// Matrix representation of the receiver.
        ///
        /// See: ``transform``.
        @inlinable public var matrix: Matrix { Matrix(Math.make3(x), Math.make3(y), Matrix.Column(origin, 1)) }

        /// *KvTransform2* representation of the receiver.
        ///
        /// See: ``matrix``.
        @inlinable public var transform: KvTransform2 { KvTransform2(matrix) }


        /// Normalizes the receiver's vectors.
        ///
        /// See: ``normalized()``.
        @inlinable
        public mutating func normalize() {
            x = Math.normalize(x)
            y = Math.normalize(y)
        }


        /// - Returns: A copy of the receiver where the vectors are normalized.
        ///
        /// See: ``normalize()``, ``safeNormalized()``.
        @inlinable public func normalized() -> Basis { Basis(x: Math.normalize(x), y: Math.normalize(y), origin: origin) }


        /// - Returns: A copy of the receiver where all the vectors are normalized if nonzero. If any vector is zero then `nil` is returned.
        ///
        /// See: ``normalized()``.
        @inlinable
        public func safeNormalized() -> Basis? {
            guard let x = Math.safeNormalize(x),
                  let y = Math.safeNormalize(y)
            else { return nil }

            return Basis(x: x, y: y, origin: origin)
        }

        

        // MARK: .Permutation

        /// Permutation of 2D basis vectors.
        public enum Permutation { case xy, yx }

    }

}
