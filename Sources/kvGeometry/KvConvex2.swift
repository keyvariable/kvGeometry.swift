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
//  KvConvex2.swift
//  kvGeometry
//
//  Created by Svyatoslav Popov on 22.09.2022.
//

//import Foundation

import kvKit



/// Implementation of convex polygon in 2D coordinate space.
///
/// Convex polygons are defined by sequence of coordinates enumerated in counterclockwise (CCW) or clockwise (CW) direction.
public struct KvConvex2<Vertex : KvVertex2Protocol> {

    public typealias Math = Vertex.Math
    public typealias Vertex = Vertex

    public typealias Vector = Math.Vector2

    public typealias Transform = KvTransform2<Math>
    public typealias AffineTransform = KvAffineTransform2<Math>



    /// Vertices producing the receiver. The elements are in CW or CCW direction.
    @inlinable
    public var vertices: AnyCollection<Vertex> {
        !isReversed ? AnyCollection(_vertices) : AnyCollection(_vertices.reversed())
    }



    /// Vertices producing the receiver. The elements are in CW or CCW direction.
    @usableFromInline
    internal var _vertices: [Vertex]

    /// A boolean value indicating whether elements of *_vertices* array are enumerated in reverse or direct order.
    @usableFromInline
    internal var isReversed: Bool


    /// Memberwise initializer.
    ///
    /// - Warning: The caller have to provide unique vertices in CCW or CW direction with non-degenerate angles.
    /// - Note: The caller is responsible for clonning provided vertices.
    @inlinable
    public init(unsafeVertices vertices: [Vertex], reverse isReversed: Bool = false) {
        self._vertices = vertices
        self.isReversed = isReversed
    }

    /// Memberwise initializer.
    ///
    /// - Warning: The caller have to provide unique vertices in CCW or CW direction with non-degenerate angles.
    /// - Note: The caller is responsible for clonning provided vertices.
    @inlinable
    public init(unsafeVertices vertices: Vertex..., reverse isReversed: Bool = false) {
        self.init(unsafeVertices: vertices, reverse: isReversed)
    }

    /// Memberwise initializer.
    ///
    /// - Warning: The caller have to provide unique vertices in CCW or CW direction with non-degenerate angles.
    /// - Note: The caller is responsible for clonning provided vertices.
    @inlinable
    public init<Vertices>(unsafeVertices vertices: Vertices, reverse isReversed: Bool = false)
    where Vertices : Sequence, Vertices.Element == Vertex
    {
        self.init(unsafeVertices: Array(vertices), reverse: isReversed)
    }


    /// Initializes a convex polygon produced from the minimum subsequence of given vertices in CCW or CW direction.
    ///
    /// - Parameter vertices: A sequence of vertices having no overlapping neighbour segments.
    ///
    /// Given vertices are filtered to drop numerically equal coordinates and join subsequent co-directional segments.
    /// Then the resulting sequence is checked for the same direction: CCW or CW.
    /// Then an instance is initialized with a valid sequence of vertices.
    ///
    /// - Note: The caller is responsible for clonning provided vertices.
    public init?<Vertices>(_ vertices: Vertices, reverse isReversed: Bool = false)
    where Vertices : Sequence, Vertices.Element == Vertex
    {
        var convexVertices: [Vertex] = .init()

        var iterator = VertexIterator(vertices)

        while let next = iterator.next() {
            switch next.direction {
            case .ccw, .cw:
                break
            case .invalid, .mixed:
                return nil
            }

            convexVertices.append(next.vertex)
        }

        self.init(unsafeVertices: convexVertices, reverse: isReversed)
    }


    /// Initializes a convex polygon produced from the minimum subsequence of given vertices in CCW or CW direction.
    ///
    /// - Parameter vertices: A sequence of vertices having no overlapping neighbour segments.
    ///
    /// Given vertices are filtered to drop numerically equal coordinates and join subsequent co-directional segments.
    /// Then the resulting sequence is checked for the same direction: CCW or CW.
    /// Then an instance is initialized with a valid sequence of vertices.
    ///
    /// - Note: The caller is responsible for clonning provided vertices.
    @inlinable
    public init?(_ vertices: Vertex..., reverse isReversed: Bool = false) {
        self.init(vertices, reverse: isReversed)
    }



    // MARK: Operations

    /// A boolean value indicating whether vertices of the valid receiver are enumerated in CCW dicrection and *isReversed* is *false* or
    /// the vertices are enumerated in CW dicrection and *isReversed* is *true*.
    ///
    ///  - SeeAlso: ``isValid``
    ///
    /// - Warning: The caller is responsible for the receiver to be valid. Otherwise the result is undefined.
    @inlinable
    public var isCCW: Bool { isSourceCCW ? !isReversed : isReversed }

    /// A boolean value indicating whether *\_vertices* property is in CCW order.
    ///
    /// - Warning: The caller is responsible for the receiver to be valid. Otherwise the result is undefined.
    @usableFromInline
    internal var isSourceCCW: Bool {
        guard _vertices.count >= 3 else { return false }

        let c0 = _vertices[0].coordinate
        let c1 = _vertices[1].coordinate
        let c2 = _vertices[2].coordinate

        let epsArg1 = Math.epsArg(c1)
        let epsArg = (epsArg1 - Math.epsArg(c0)).cross(Math.epsArg(c2) - epsArg1)

        return KvIsPositive(Math.cross(c1 - c0, c2 - c1).z, eps: epsArg.tolerance)
    }


    /// A boolean value indicating whether angles on all vertices are not degenerate and the directions are the same on all vertices.
    ///
    ///  - SeeAlso: ``isCCW``
    public var isValid: Bool {
        var iterator = VertexIterator(_vertices)
        var direction: Direction = .invalid

        while let next = iterator.next() {
            direction = next.direction
        }

        switch direction {
        case .ccw, .cw:
            return true
        case .invalid, .mixed:
            return false
        }
    }


    /// Vertices enumerated in CCW direction.
    ///
    /// - Warning: The caller is responsible for the receiver to be valid. Otherwise the result is undefined.
    @inlinable
    public var ccwVertices: AnyCollection<Vertex> {
        isSourceCCW ? AnyCollection(_vertices) : AnyCollection(_vertices.reversed())
    }
    /// Vertices enumerated in CW direction.
    ///
    /// - Warning: The caller is responsible for the receiver to be valid. Otherwise the result is undefined.
    @inlinable
    public var cwVertices: AnyCollection<Vertex> {
        !isSourceCCW ? AnyCollection(_vertices) : AnyCollection(_vertices.reversed())
    }


    /// In the DEBUG configuration invokes assertionFailure() with an informative message if the receiver contains any invalid vartex.
    /// In the RELEASE configuration does nothing
#if DEBUG
    public func assert() {
        var iterator = VertexIterator(_vertices)
        let direction: Direction

        do {
            guard let first = iterator.next()
            else { return assertionFailure("The convex polygon doesn't contain 3 vertices with non-degenerate angle between") }

            switch first.direction {
            case .ccw, .cw:
                direction = first.direction
            case .invalid:
                return assertionFailure("Invalid direction at `\(first.vertex)` vertex")
            case .mixed:
                return assertionFailure("Internal inconsistency: mixed direction at `\(first.vertex)` first vertex")
            }
        }

        while let next = iterator.next() {
            switch next.direction {
            case .ccw:
                guard direction == .ccw
                else { return assertionFailure("Unexpected CCW direction at `\(next.vertex)` vertex of CW convex polygon") }
            case .cw:
                guard direction == .cw
                else { return assertionFailure("Unexpected CW direction at `\(next.vertex)` vertex of CCW convex polygon") }
            case .invalid:
                return assertionFailure("Invalid direction at `\(next.vertex)` vertex")
            case .mixed:
                switch direction {
                case .ccw:
                    return assertionFailure("Unexpected CW direction at `\(next.vertex)` vertex of CCW convex polygon")
                case .cw:
                    return assertionFailure("Unexpected CCW direction at `\(next.vertex)` vertex of CW convex polygon")
                case .invalid:
                    return assertionFailure("Internal inconsistency: invalid inferred direction")
                case .mixed:
                    return assertionFailure("Internal inconsistency: mixed inferred direction")
                }
            }
        }
    }
#else // !DEBUG
    @inlinable public func assert() { }
#endif // !DEBUG



    /// Reverses the receiver's vertices.
    @inlinable public mutating func reverse() { isReversed.toggle() }


    /// - Returns: A convex polygon matching the receiver but having reversed vertices.
    @inlinable public func reversed() -> Self { Self(unsafeVertices: _vertices.map { $0.clone() }, reverse: !isReversed) }


    /// - Returns: Copy of the receiver where vertices are cloned.
    @inlinable public func clone() -> Self { Self(unsafeVertices: _vertices.map { $0.clone() }, reverse: isReversed) }


    /// Flips and reverses the receiver's vertices.
    @inlinable
    public mutating func flip() {
        _vertices.indices.forEach {
            _vertices[$0].flip()
        }
        reverse()
    }


    /// - Returns: A convex polygon matching the receiver but having flipped and reversed vertices.
    @inlinable public func flipped() -> Self { Self(unsafeVertices: _vertices.map { $0.flipped() }, reverse: !isReversed) }


    /// Translates the receiver's vertices.
    @inlinable
    public mutating func translate(by offset: Vector) {
        _vertices.indices.forEach {
            _vertices[$0] += offset
        }
    }


    /// - Returns: A copy of the receiver where all vertices are translated by given offset.
    @inlinable
    public func translated(by offset: Vector) -> Self {
        Self(unsafeVertices: _vertices.map { $0 + offset }, reverse: isReversed)
    }


    /// Applies given transformation to all the receiver's vertices.
    @inlinable public mutating func apply(_ t: Transform) { self = t * self }

    /// Applies given transformation to all the receiver's vertices.
    @inlinable public mutating func apply(_ t: AffineTransform) { self = t * self }


    public typealias SplitResult = (front: Self?, back: Self?)

    /// - Returns: Front and back parts of the receiver relative to given line.
    public func split(by line: KvLine2<Math>) -> SplitResult {
        typealias Element = (vertex: Vertex, location: KvLine2<Math>.Location)
        /// `matchesSource` indicates whether all the receivers vertices are collected.
        typealias Accumulator = (vertices: [Vertex], matchesSource: Bool)

        var iterator = _vertices
            .lazy.map { ($0, line.location(of: $0.coordinate)) }
            .makeIterator()

        guard let first = iterator.next() else { return (nil, nil) }

        var front: Accumulator = (.init(), true)
        var back: Accumulator = (.init(), true)


        func Process(prev: Element, next: Element) {

            func AppendIntersection() {
                let ray = KvRay2(from: prev.vertex, to: next.vertex.coordinate)
                let v = ray.intersection(with: line)!

                front.vertices.append(v)
                front.matchesSource = false

                back.vertices.append(v)
                back.matchesSource = false
            }


            switch (prev.location, next.location) {
            case (.negative, .positive), (.positive, .negative):
                AppendIntersection()
            default:
                break
            }

            switch next.location {
            case .negative:
                back.vertices.append(next.vertex)
                front.matchesSource = false
            case .positive:
                front.vertices.append(next.vertex)
                back.matchesSource = false
            case .neutral:
                front.vertices.append(next.vertex)
                back.vertices.append(next.vertex)
            }
        }


        func MakeConvex(with accumulator: Accumulator) -> Self? {
            switch accumulator.matchesSource {
            case true:
                return self

            case false:
                guard accumulator.vertices.count >= 3 else { return nil }

                return Self(accumulator.vertices, reverse: isReversed)
            }
        }


        var prev = first

        while let next = iterator.next() {
            defer { prev = next }

            Process(prev: prev, next: next)
        }

        Process(prev: prev, next: first)

        return (front: MakeConvex(with: front), back: MakeConvex(with: back))
    }



    // MARK: Operators

    /// - Returns: Result of given transformation applied to *rhs*.
    @inlinable
    public static func *(lhs: Transform, rhs: Self) -> Self {
        Self(unsafeVertices: rhs._vertices.map { lhs * $0 }, reverse: rhs.isReversed)
    }

    /// - Returns: Result of given transformation applied to *rhs*.
    @inlinable
    public static func *(lhs: AffineTransform, rhs: Self) -> Self {
        Self(unsafeVertices: rhs._vertices.map { lhs * $0 }, reverse: rhs.isReversed)
    }



    // MARK: Auxiliaries

    /// - Returns: The direction of linerwise closed path on given vertices.
    public static func direction<Vertices>(of vertices: Vertices) -> Direction
    where Vertices : Sequence, Vertices.Element == Vertex
    {
        var iterator = VertexIterator(vertices)
        var direction: Direction = .invalid

        while let vertex = iterator.next() {
            direction = vertex.direction
        }

        return direction
    }

    /// - Returns: The direction of linerwise closed path on given vertices.
    @inlinable public static func direction(of vertices: Vertex...) -> Direction { direction(of: vertices) }

}



// MARK: : KvNumericallyEquatable

extension KvConvex2 : KvNumericallyEquatable where Vertex : KvNumericallyEquatable {

    /// - Returns: A boolean value indicating whether the receiver and *rhs* are numerically equal.
    @inlinable
    public func isEqual(to rhs: Self) -> Bool {
        isReversed == rhs.isReversed
        ? __isEqual(_vertices, rhs._vertices)
        : __isEqual(_vertices, rhs._vertices.reversed())
    }

    /// - Returns: A boolean value indicating whether the receiver and *rhs* are numerically equal.
    @inlinable
    public func isEqual<V>(to rhs: KvConvex2<V>) -> Bool
    where V : KvVertex2Protocol, V.Math == Vertex.Math
    {
        isReversed == rhs.isReversed
        ? __isEqual(_vertices, rhs._vertices)
        : __isEqual(_vertices, rhs._vertices.reversed())
    }


    /// - Returns: A boolean value indicating whether *lhs* is numerically equal to cyclically shifted *rhs* by some number of elements.
    @usableFromInline
    internal func __isEqual<LHS, RHS>(_ lhs: LHS, _ rhs: RHS) -> Bool
    where LHS : Sequence, LHS.Element : KvVertex2Protocol, LHS.Element.Math == Math,
          RHS : Sequence, RHS.Element : KvVertex2Protocol, RHS.Element.Math == Math
    {
        guard let lFirst = lhs.first(where: { _ in true }) else { return false }
        guard let (offset, _) = rhs.enumerated().first(where: { Math.isEqual($0.element.coordinate, lFirst.coordinate) }) else { return false }

        let rhsShifted = [ AnySequence(rhs.dropFirst(offset + 1)), AnySequence(rhs.prefix(offset)) ].joined()

        return lhs.dropFirst(1)
            .lazy.map { $0.coordinate }
            .elementsEqual(rhsShifted.lazy.map { $0.coordinate }, by: Math.isEqual(_:_:))
    }

}



// MARK: : Equatable

extension KvConvex2 : Equatable where Vertex : Equatable { }



// MARK: : Hashable

extension KvConvex2 : Hashable where Vertex : Hashable { }



// MARK: Auxliliaries

extension KvConvex2 {

    // MARK: .Direction

    /// Direction at all the polygon vertices.
    public enum Direction : Hashable {
        /// CounterClockWise.
        case ccw
        /// ClockWise.
        case cw
        // Other cases
        case mixed, invalid
    }



    // MARK: .LocalDirection

    /// Direction at a vertex and it's neighbours.
    public enum LocalDirection : Hashable {
        /// Counterclockwise.
        case ccw
        /// Clockwise.
        case cw
        /// Angle on three vertices is zero or *pi*.
        case degenerate
    }



    // MARK: .VertexIteratorElement

    public struct VertexIteratorElement {

        /// Coordinate of vertex.
        public var vertex: Vertex
        /// Offset from the receiver's *coordinate* to the next coordinate.
        public var step: Step
        /// Direction of subpath until the coordinate.
        public var direction: Direction


        // MARK: Fabrics

        static func from(_ source: PolygonVertexIteratorElement, direction: Direction) -> Self {
            Self(vertex: source.vertex,
                 step: source.step,
                 direction: direction)
        }


        // MARK: .Step

        public struct Step : KvNumericallyEquatable, Hashable {

            public let vector: Vector
            public let epsArg: Math.EpsArg2


            /// Memberwise initializer
            @inlinable
            public init(_ vector: Vector, epsArg: Math.EpsArg2) {
                self.vector = vector
                self.epsArg = epsArg
            }

            /// Initializes step with default tolerance argument.
            @inlinable public init(_ vector: Vector) { self.init(vector, epsArg: Math.epsArg(vector)) }

            /// Initializes step with a vector between two given coordinates and default tolerance argument.
            @inlinable public init(from: Vector, to: Vector) { self.init(to - from, epsArg: Math.epsArg(to) - Math.epsArg(from)) }


            // MARK: Auxiliaries

            @inlinable public static var zero: Self { Self(.zero, epsArg: .zero) }


            // MARK: Operators

            @inlinable public static prefix func -(rhs: Self) -> Self { Self(-rhs.vector, epsArg: rhs.epsArg) }

            @inlinable public static func +(lhs: Self, rhs: Self) -> Self { Self(lhs.vector + rhs.vector, epsArg: lhs.epsArg + rhs.epsArg) }


            // MARK: : KvNumericallyEquatable

            @inlinable
            public func isEqual(to rhs: Self) -> Bool {
                Math.isEqual(vector, rhs.vector, eps: (epsArg - rhs.epsArg).tolerance)
            }

        }

    }


    // MARK: .PolygonVertexIteratorElement

    struct PolygonVertexIteratorElement {

        typealias Step = VertexIteratorElement.Step


        /// Coordinate of vertex.
        var vertex: Vertex
        /// Offset from the receiver's *coordinate* to the next coordinate.
        var step: Step
        /// Direction at the coordinate and it's neighbours.
        var direction: KvConvex2.LocalDirection

    }


    // MARK: .VertexIterator

    /// Filters input vertices to minimum subsequence producing a convex polygon.
    /// For each vertex calculates the subpath direction until the vertex. So last direction matches with the direction of whole convex polygon on closed path from the vertices.
    /// If direction of a subpath is not .ccw or .cw, then the rest of vertices is ignored.
    public struct VertexIterator<Vertices> : IteratorProtocol
    where Vertices : Sequence, Vertices.Element == Vertex
    {

        public init(_ vertices: Vertices) {
            nextBlock = FSM.initialStateBlock(vertices)
        }


        /// A FSM state as a block.
        private var nextBlock: FSM.Block


        // MARK: : IteratorProtocol

        public typealias Element = VertexIteratorElement


        /// - Returns: Next path vertex. It's *direction* property is the direction of subpath on all the returned vertices.
        ///
        /// - Note: *Direction* property of the last element is the direction of the convex polygon.
        public mutating func next() -> Element? { nextBlock(&self) }


        // MARK: .FSM

        private class FSM {

            typealias Block = (inout VertexIterator) -> Element?


            private init(_ vertices: Vertices) {
                iterator = .init(vertices)
            }


            private var iterator: PolygonVertexIterator


            // MARK: States

            static func initialStateBlock(_ vertices: Vertices) -> Block {
                let fsm = FSM(vertices)

                return { _self in
                    guard let first = fsm.iterator.next() else { return nil }

                    let pathDirection: Direction

                    switch first.direction {
                    case .ccw:
                        pathDirection = .ccw
                    case .cw:
                        pathDirection = .cw
                    case .degenerate:
                        _self.nextBlock = endStateBlock()
                        return .from(first, direction: .invalid)
                    }

                    _self.nextBlock = regularStateBlock(fsm, pathDirection)

                    return .from(first, direction: pathDirection)
                }
            }


            private static func regularStateBlock(_ fsm: FSM, _ pathDirection: Direction) -> Block {
                return { _self in
                    guard let next = fsm.iterator.next() else { return nil }

                    switch next.direction {
                    case .ccw:
                        guard pathDirection == .ccw else {
                            _self.nextBlock = endStateBlock()
                            return .from(next, direction: .mixed)
                        }

                    case .cw:
                        guard pathDirection == .cw else {
                            _self.nextBlock = endStateBlock()
                            return .from(next, direction: .mixed)
                        }
                    case .degenerate:
                        _self.nextBlock = endStateBlock()
                        return .from(next, direction: .invalid)
                    }

                    return .from(next, direction: pathDirection)
                }
            }


            private static func endStateBlock() -> Block {
                return { _ in
                    nil
                }
            }

        }


        // MARK: .PolygonVertexIterator

        /// Filters input and returns minimum subsequence of vertices producing a polygon with non-degenerate angles. Also path direction is returned for each vertex.
        struct PolygonVertexIterator : IteratorProtocol {

            init(_ vertices: Vertices) {
                nextBlock = FSM.initialStateBlock(vertices)
            }


            private typealias Step = PolygonVertexIteratorElement.Step


            /// A FSM state as a block.
            private var nextBlock: FSM.Block


            // MARK: .VertexContainer

            private struct VertexContainer {

                let vertex: Vertices.Element
                let epsArg: Math.EpsArg2


                /// Memberwise initializer
                init(_ vertex: Vertices.Element, epsArg: Math.EpsArg2) {
                    self.vertex = vertex
                    self.epsArg = epsArg
                }


                init(_ vertex: Vertices.Element) { self.init(vertex, epsArg: Math.epsArg(vertex.coordinate)) }


                // MARK: Operators

                static func -(lhs: Self, rhs: Self) -> Step { Step(lhs.vertex.coordinate - rhs.vertex.coordinate, epsArg: lhs.epsArg - rhs.epsArg) }

            }


            // MARK: .LocalDirection

            /// Direction at a vertex and it's neighbours.
            private enum LocalDirection : Hashable {

                /// Counterclockwise.
                case ccw
                /// Clockwise.
                case cw

                case frontOrUndefined, backward


                // MARK: Init

                /// - Parameter s1: Vector from first to second vertex.
                /// - Parameter s2: Vector from second to third vertex.
                ///
                /// An instance is initialized with the direction of three vertices.
                public init(steps s1: Step, _ s2: Step) {
                    var isNegative = false

                    if KvIsPositive(Math.cross(s1.vector, s2.vector).z, eps: s1.epsArg.cross(s2.epsArg).tolerance, alsoIsNegative: &isNegative) {
                        self = .ccw
                    }
                    else if isNegative {
                        self = .cw
                    }
                    else { self = (KvIsNotNegative(Math.dot(s1.vector, s2.vector), eps: s1.epsArg.dot(s2.epsArg).tolerance)
                                   ? .frontOrUndefined : .backward) }
                }


                /// An instance is initialized with the direction of three vertices.
                @inlinable
                public init(vertices v1: VertexContainer, _ v2: VertexContainer, _ v3: VertexContainer) {
                    self.init(steps: v2 - v1, v3 - v2)
                }

            }


            // MARK: : IteratorProtocol

            typealias Element = PolygonVertexIteratorElement


            /// - Returns: Next path vertex and the direction of subpath on all the returned vertices.
            mutating func next() -> Element? { nextBlock(&self) }


            // MARK: .FSM

            private class FSM {

                typealias Block = (inout PolygonVertexIterator) -> Element?


                private init(_ vertices: Vertices) {
                    iterator = .init(vertices)
                }


                private var iterator: DistinctCoordinateIterator


                // MARK: States

                static func initialStateBlock(_ vertices: Vertices) -> Block {
                    let fsm = FSM(vertices)

                    return { _self in
                        guard let start = fsm.nextVertex() else {
                            _self.nextBlock = endStateBlock()
                            return nil
                        }

                        guard var p1 = fsm.nextVertex() else {
                            _self.nextBlock = endStateBlock()
                            return .init(vertex: start.vertex, step: .zero, direction: .degenerate)
                        }

                        var s1 = p1 - start

                        // Enumerating vertices until first non-degenerate case.
                        while let p2 = fsm.nextVertex() {
                            let s2 = p2 - p1

                            switch LocalDirection(steps: s1, s2) {
                            case .ccw:
                                _self.nextBlock = regularStateBlock(
                                    fsm, start, s1,
                                    Element(vertex: p1.vertex, step: s2, direction: .ccw),
                                    p2)
                                return _self.nextBlock(&_self)

                            case .cw:
                                _self.nextBlock = regularStateBlock(
                                    fsm, start, s1,
                                    Element(vertex: p1.vertex, step: s2, direction: .cw),
                                    p2)
                                return _self.nextBlock(&_self)

                            case .frontOrUndefined:
                                s1 = s1 + s2
                                p1 = p2

                            case .backward:
                                _self.nextBlock = endStateBlock()
                                return Element(vertex: p1.vertex, step: s2, direction: .degenerate)
                            }
                        }

                        // Two point case of all-front case.
                        _self.nextBlock = endStateBlock()
                        return Element(vertex: p1.vertex, step: -s1, direction: .degenerate)
                    }
                }


                /// - Parameter element: Pending element to retrun.
                private static func regularStateBlock(_ fsm: FSM, _ first: VertexContainer, _ firstStep: Step, _ element: Element, _ p2: VertexContainer) -> Block {
                    var element = element
                    var p2 = p2

                    return { _self in
                        while let p3 = fsm.nextVertex() {
                            let s3 = p3 - p2

                            defer { p2 = p3 }

                            switch LocalDirection(steps: element.step, s3) {
                            case .ccw:
                                defer { element = Element(vertex: p2.vertex, step: s3, direction: .ccw) }
                                return element
                            case .cw:
                                defer { element = Element(vertex: p2.vertex, step: s3, direction: .cw) }
                                return element
                            case .frontOrUndefined:
                                element.step = element.step + s3
                            case .backward:
                                _self.nextBlock = pendingVertexStateBlock(Element(vertex: p2.vertex, step: s3, direction: .degenerate))
                                return element
                            }
                        }

                        // Closing the path
                        do {
                            let s0 = first - p2

                            switch LocalDirection(steps: element.step, s0) {
                            case .ccw:
                                _self.nextBlock = lastElementStateBlock(Element(vertex: p2.vertex, step: s0, direction: .ccw), first, s0, firstStep)

                            case .cw:
                                _self.nextBlock = lastElementStateBlock(Element(vertex: p2.vertex, step: s0, direction: .cw), first, s0, firstStep)

                            case .frontOrUndefined:
                                // *P2* point is ignored in this case.
                                element.step = element.step + s0
                                _self.nextBlock = lastElementStateBlock(element, first, element.step, firstStep)
                                return _self.nextBlock(&_self)

                            case .backward:
                                _self.nextBlock = pendingVertexStateBlock(Element(vertex: p2.vertex, step: s0, direction: .degenerate))
                            }

                            return element
                        }
                    }
                }


                /// Given element will be posted once, then the end state is entered.
                private static func lastElementStateBlock(_ element: Element, _ p1: VertexContainer, _ s1: Step, _ s2: Step) -> Block {
                    var element = element

                    return { _self in
                        switch LocalDirection(steps: s1, s2) {
                        case .ccw:
                            _self.nextBlock = pendingVertexStateBlock(Element(vertex: p1.vertex, step: s2, direction: .ccw))

                        case .cw:
                            _self.nextBlock = pendingVertexStateBlock(Element(vertex: p1.vertex, step: s2, direction: .cw))

                        case .frontOrUndefined:
                            // *Start* point is ignored in this case.
                            element.step = element.step + s2
                            _self.nextBlock = endStateBlock()

                        case .backward:
                            _self.nextBlock = pendingVertexStateBlock(Element(vertex: p1.vertex, step: s2, direction: .degenerate))
                        }

                        return element
                    }
                }


                private static func pendingVertexStateBlock(_ element: Element) -> Block {
                    return { _self in
                        _self.nextBlock = endStateBlock()
                        return element
                    }
                }


                /// Nothing is iterated, *nil* is always returned.
                private static func endStateBlock() -> Block {
                    return { _ in
                        nil
                    }
                }


                // MARK: Operations

                private func nextVertex() -> VertexContainer? { iterator.next().map(VertexContainer.init(_:)) }

            }

        }


        // MARK: .DistinctCoordinateIterator

        /// Produces sequene of numerically inequal coordinates from given arbitrary sequence.
        struct DistinctCoordinateIterator : IteratorProtocol {

            init(_ vertices: Vertices) {
                nextBlock = FSM.initialStateBlock(vertices)
            }


            /// A FSM state as a block.
            private var nextBlock: FSM.Block


            // MARK: : IteratorProtocol

            typealias Element = Vertices.Element


            mutating func next() -> Element? { nextBlock(&self) }


            // MARK: .FSM

            private class FSM {

                typealias Block = (inout DistinctCoordinateIterator) -> Element?


                private init(_ vertices: Vertices) {
                    iterator = vertices.makeIterator()
                }


                private var iterator: Vertices.Iterator


                // MARK: States

                static func initialStateBlock(_ vertices: Vertices) -> Block {
                    let fsm = FSM(vertices)

                    return { _self in
                        guard let first = fsm.iterator.next() else { return nil }

                        _self.nextBlock = regularStateBlock(fsm, first)

                        return first
                    }
                }


                private static func regularStateBlock(_ fsm: FSM, _ last: Vertices.Element) -> Block {
                    var last = last

                    return { _self in
                        while let next = fsm.iterator.next() {
                            if Math.isInequal(next.coordinate, last.coordinate) {
                                last = next
                                return next
                            }
                        }

                        return nil
                    }
                }

            }

        }

    }

}


extension KvConvex2.VertexIteratorElement : KvNumericallyEquatable {

    /// - Returns: A boolean value indicating whether the receiver adn *rhs* are numerically equal.
    @inlinable
    public func isEqual(to rhs: Self) -> Bool {
        direction == rhs.direction
        && KvConvex2.Math.isEqual(vertex.coordinate, rhs.vertex.coordinate)
        && step.isEqual(to: rhs.step)
    }

}


extension KvConvex2.VertexIteratorElement : Equatable where Vertex : Equatable { }


extension KvConvex2.VertexIteratorElement : Hashable where Vertex : Hashable { }



extension KvConvex2.PolygonVertexIteratorElement : KvNumericallyEquatable {

    /// - Returns: A boolean value indicating whether the receiver adn *rhs* are numerically equal.
    @inlinable
    public func isEqual(to rhs: Self) -> Bool {
        direction == rhs.direction
        && KvConvex2.Math.isEqual(vertex.coordinate, rhs.vertex.coordinate)
        && step.isEqual(to: rhs.step)
    }

}


extension KvConvex2.PolygonVertexIteratorElement : Equatable where Vertex : Equatable { }


extension KvConvex2.PolygonVertexIteratorElement : Hashable where Vertex : Hashable { }
