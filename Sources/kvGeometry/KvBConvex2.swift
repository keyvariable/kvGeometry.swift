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
//  KvBConvex2.swift
//  kvGeometry
//
//  Created by Svyatoslav Popov on 23.09.2022.
//

import kvKit



/// Implementation of bounding convex polygon equal to intersection of left halfspaces produced by given oriented lines in 2D coordinate space.
public struct KvBConvex2<Math : KvMathScope> {

    public typealias Math = Math

    public typealias Vector = Math.Vector2
    public typealias Coordinate = Vector

    public typealias Transform = KvTransform2<Math>
    public typealias AffineTransform = KvAffineTransform2<Math>

    public typealias Line = KvLine2<Math>



    public let lines: [Line]



    /// Memberwise initializer.
    ///
    /// - Attention: The caller have to provide valid agruments.
    @inlinable public init(unsafeLines lines: [Line]) { self.lines = lines }

    /// Memberwise initializer.
    ///
    /// - Attention: The caller have to provide valid agruments.
    @inlinable public init(unsafeLines lines: Line...) { self.init(unsafeLines: lines) }

    /// Memberwise initializer.
    ///
    /// - Attention: The caller have to provide valid agruments.
    @inlinable
    public init<Lines>(unsafeLines lines: Lines)
    where Lines : Sequence, Lines.Element == Line
    {
        self.init(unsafeLines: Array(lines))
    }


    /// Initializes an instance matching the shape of given convex polygon.
    public init<V>(_ convex: KvConvex2<V>)
    where V : KvVertex2Protocol, V.Math == Math
    {
        var lines: [Line] = .init()

        var iterator = convex.ccwVertices
            .lazy.map { $0.coordinate }
            .makeIterator()

        // No validation for line orientations assuming the convex vertices are valid.
        if let first = iterator.next() {
            var prev = first

            while let next = iterator.next() {
                lines.append(Line(prev, next))
                prev = next
            }

            lines.append(Line(prev, first))
        }

        self.lines = lines
    }


    /// Initializes an instance matching shape of given bounding box.
    @inlinable
    public init(_ aabr: KvAABB2<Math>) {
        lines = [
            Line(normal: [ -1,  0 ], c:  aabr.min.x),
            Line(normal: [  1,  0 ], c: -aabr.max.x),
            Line(normal: [  0, -1 ], c:  aabr.min.y),
            Line(normal: [  0,  1 ], c: -aabr.max.y),
        ]
    }


    /// - Parameter coordinates: Coordinates of convex polygon vertices in CCW or CW direction.
    public init?<Coordinates>(_ coordinates: Coordinates)
    where Coordinates : Sequence, Coordinates.Element == Coordinate
    {
        typealias Convex = KvConvex2<KvPosition2<Math, Void>>

        var lines: [Line] = .init()

        var iterator = Convex.VertexIterator(coordinates.lazy.map { .init($0) })

        if let first = iterator.next() {
            let directionFactor: Math.Scalar

            switch first.direction {
            case .ccw:
                directionFactor = 1
            case .cw:
                directionFactor = -1
            case .mixed, .invalid:
                return nil
            }


            func AppendLine(from element: Convex.VertexIteratorElement) {
                lines.append(Line(in: directionFactor * element.step.vector, at: element.vertex.coordinate))
            }


            AppendLine(from: first)

            while let next = iterator.next() {
                guard next.direction == first.direction else { return nil }

                AppendLine(from: next)
            }
        }

        self.lines = lines
    }


    /// - Parameter coordinates: Coordinates of convex polygon vertices in CCW or CW direction.
    @inlinable
    public init?(_ points: Coordinate...) {
        switch points.count {
        case 0, 1, 2:
            return nil
        case 3:
            self.init(triangle: points[0], points[1], points[2])
        default:
            self.init(points)
        }
    }


    @inlinable
    public init?(triangle c0: Coordinate, _ c1: Coordinate, _ c2: Coordinate) {
        let line1 = Line(c0, c1)
        guard !line1.isDegenerate else { return nil }

        var isNegative = false

        if KvIsPositive(line1.signedOffset(to: c2), alsoIsNegative: &isNegative) {
            lines = [ line1, Line(c1, c2), Line(c2, c0) ]
        }
        else if isNegative {
            lines = [ -line1, Line(c0, c2), Line(c2, c1) ]
        }
        else { return nil }
    }


    // MARK: Operations

    /// - Returns: A boolean value indicating whether the given coordinate is not outside the receiver.
    ///
    /// - SeeAlso: ``containsInside(_:)``
    @inlinable
    public func contains(_ c: Coordinate) -> Bool {
        lines.allSatisfy {
            KvIsNotNegative($0.signedOffset(to: c))
        }
    }


    /// - Returns: A boolean value indicating whether the given coordinate is inside the receiver.
    ///
    /// - SeeAlso: ``contains(_:)``
    @inlinable
    public func containsInside(_ c: Coordinate) -> Bool {
        lines.allSatisfy {
            KvIsNotNegative($0.signedOffset(to: c))
        }
    }


    /// - Returns: Range of X-coordinates inside the receiver at given Y-coordinate.
    public func segment(y: Math.Scalar) -> ClosedRange<Math.Scalar>? {
        var lowerBound: Math.Scalar = -.infinity
        var upperBound: Math.Scalar = .infinity

        var iterator = lines.makeIterator()

        while let line = iterator.next() {
            switch line.x(y: y) {
            case .some(let x):
                // If line intersects with horizontal then it isn't horizontal.
                line.normal.x < 0
                ? (upperBound = Swift.min(upperBound, x))
                : (lowerBound = Swift.max(lowerBound, x))

            case .none:
                // line.normal.y = ±1
                guard KvIs(line.c, greaterThanOrEqualTo: y * (line.normal.y > 0 ? -1 : 1)) else { return nil }
            }
        }

        return upperBound >= lowerBound ? lowerBound ... upperBound : nil
    }

}



// MARK: : KvNumericallyEquatable

extension KvBConvex2 : KvNumericallyEquatable {

    /// - Returns: A boolean value indicating whether the receiver and *rhs* are numerically equal.
    @inlinable
    public func isEqual(to rhs: Self) -> Bool {
        guard lines.count == rhs.lines.count else { return false }

        // TODO: Improve performance. Sort the lines in CCW order and compare BConvexes as sorted arrays of lines.
        var rLines = rhs.lines

        for line in lines {
            guard let index = rLines.firstIndex(where: { line.isEqual(to: $0) }) else { return false }
            rLines.remove(at: index)
        }

        return rLines.isEmpty
    }

}



// MARK: : Equatable

extension KvBConvex2 : Equatable { }



// MARK: : Hashable

extension KvBConvex2 : Hashable { }
