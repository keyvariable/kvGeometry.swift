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
//  KvCsgBspNode.swift
//  kvCSG
//
//  Created by Svyatoslav Popov on 30.09.2022.
//

import simd

import kvKit

import kvGeometry



/// BSP node containing convex polygons on a plane.
/// The plane is XY plane in the local coodinate space defined by world transformation.
public class KvCsgBspNode<Math, Vertex, Payload> : KvCsgPolygonal
where Math : KvMathScope, Vertex : KvVertex3Protocol, Vertex.Math == Math
{

    public typealias Math = Math
    public typealias Vertex = Vertex
    public typealias Payload = Payload

    public typealias Transform = KvCsgTransform<Math>

    public typealias Node = KvCsgBspNode
    public typealias Polygon2 = KvCsgPolygon2<Vertex, Payload>
    public typealias Polygon3 = KvCsgPolygon3<Vertex, Payload>



    @inlinable public var polygons: Polygons { _polygons }

    /// Child node where all polygons are in front of the receiver's plane.
    @inlinable public var front: Node? { _front }
    /// Child node where all polygons are behind of the receiver's plane.
    @inlinable public var back: Node? { _back }



    /// Memberwise initializer.
    @usableFromInline
    internal required init(polygons: Polygons, front: Node? = nil, back: Node? = nil) {
        _polygons = polygons
        _front = front
        _back = back
    }


    @inlinable
    public convenience init(with polygon: Polygon3) {
        self.init(polygons: Polygons(polygon))
    }


    /// Initializes a tree of BSP nodes for given polygons.
    public convenience init?<Polygons>(with polygons: Polygons)
    where Polygons : Sequence, Polygons.Element == Polygon3 {
        guard let first = polygons.first(where: { _ in true }) else { return nil }

        self.init(with: first)

        insert(polygons.dropFirst())
    }


    /// Initializes empty node with given transform.
    @inlinable
    public required convenience init(worldTransform: Transform) {
        self.init(polygons: Polygons(worldTransform: worldTransform))
    }


    /// Initializes empty node with world transformation defined by given plane.
    @inlinable
    public required convenience init(plane: KvPlane3<Math>) {
        self.init(worldTransform: Transform(from: plane))
    }



    @usableFromInline
    internal var _polygons: Polygons

    @usableFromInline
    internal var _front: Node?
    @usableFromInline
    internal var _back: Node?



    // MARK: .Polygons

    /// The node's polygons. CCW and CW means counterclockwise and clockwise vertex direction.
    public struct Polygons {

        /// Transformation from the local coordinates to world coordinates.
        public var worldTransform: Transform


        /// A boolean value indicating whether the geometry to be flipped and transformation to be premultiplied by the negative scale matrix.
        @usableFromInline
        internal var isInverted: Bool


        /// Memberwise initializer.
        @inlinable
        public init(worldTransform: Transform, ccw: [Polygon2] = [ ], cw: [Polygon2] = [ ], isInverted: Bool = false) {
            self.worldTransform = worldTransform
            self._ccw = ccw
            self._cw = cw
            self.isInverted = isInverted

            assert()
        }


        @inlinable
        public init(worldTransform: Transform, _ polygon: Polygon2, isInverted: Bool = false) {
            switch polygon.isCCW {
            case true:
                self.init(worldTransform: worldTransform, ccw: [ polygon ], isInverted: isInverted)
            case false:
                self.init(worldTransform: worldTransform, cw: [ polygon ], isInverted: isInverted)
            }
        }


        @inlinable
        public init(_ polygon: Polygon3, isInverted: Bool = false) {
            self.init(worldTransform: polygon.worldTransform, polygon.local, isInverted: isInverted)
        }


        @inlinable
        public init<Polygons>(worldTransform: Transform, _ polygons: Polygons, isInverted: Bool = false)
        where Polygons : Sequence, Polygons.Element == Polygon2 {
            self.isInverted = isInverted
            self.worldTransform = worldTransform

            _ccw = .init()
            _cw = .init()

            polygons.forEach { insert($0) }
        }


        @usableFromInline
        internal var _ccw, _cw: [Polygon2]


        // MARK: Operations

        @inlinable public var isEmpty: Bool { _ccw.isEmpty && _cw.isEmpty }


        /// Performs validation with the standard *assert()* function.
        @inlinable
        public func assert() {
            Swift.assert(_ccw.allSatisfy { $0.isCCW })
            Swift.assert(_cw.allSatisfy { !$0.isCCW })
        }


        @inlinable
        public func forEach(_ body: (Polygon3) throws -> Void) rethrows {
            switch isInverted {
            case false:
                try _ccw.forEach { try body(Polygon3(local: $0, worldTransform: worldTransform)) }
                try _cw.forEach { try body(Polygon3(local: $0, worldTransform: worldTransform)) }

            case true:
                let t = worldTransform * Polygons.flipTransform

                try _ccw.forEach { try body(Polygon3(local: $0, worldTransform: t).flipped()) }
                try _cw.forEach { try body(Polygon3(local: $0, worldTransform: t).flipped()) }
            }
        }


        /// Inserts given polygon.
        @inlinable
        public mutating func insert(_ polygon: Polygon2) {
            switch polygon.isCCW {
            case true:
                _ccw.append(polygon)
            case false:
                _cw.append(polygon)
            }
        }


        /// - Returns: A deep copy of the receiver.
        @inlinable
        public func clone() -> Self {
            Self(worldTransform: worldTransform,
                 ccw: _ccw.map { $0.clone() },
                 cw: _cw.map { $0.clone() },
                 isInverted: isInverted)
        }


        /// Applies given transformation to the receiver.
        @inlinable
        mutating public func apply(_ t: KvTransform3<Math>) {
            let (t, scale) = Transform.from(t)

            worldTransform = t * worldTransform

            if let scale = scale {
                KvArrayKit.mutate(&_ccw) { $0.apply(scale) }
                KvArrayKit.mutate(&_cw) { $0.apply(scale) }
            }
        }


        /// - Returns: Flips all the receiver's polygons.
        @inlinable
        public mutating func invert() {
            isInverted.toggle()

            worldTransform = worldTransform * Polygons.flipTransform
        }


        /// Removes all the receiver's polygons.
        @inlinable
        public mutating func removeAll() {
            _ccw = .init()
            _cw = .init()
        }


        /// Removes (in-place) polygons or it's parts below z = 0 plane defined by given world transformation *t*.
        ///
        /// - Returns: The removed polygons.
        ///
        /// - SeeAlso: ``clip(by:)``
        @inlinable
        public mutating func drop(below t: Transform) -> Self {
            switch worldTransform.localPlaneIntersection(withPlaneFrom: t) {
            case .some(let line):

                func FilterBackGeometry(_ polygons: inout [Polygon2]) -> [Polygon2] {
                    var removed: [Polygon2] = .init()

                    KvArrayKit.mutateAndFilter(&polygons) { polygon in
                        switch polygon.split(by: line) {
                        case (.some(let front), .some(let back)):
                            polygon = front
                            removed.append(back)
                        case (.some, .none):
                            // Assuming the front part is equal to the polygon.
                            break
                        case (.none, .some(let back)):
                            removed.append(back)
                            return false
                        case (.none, .none):
                            // - Inconsistency: Degenerate polygon. Just removing it.
                            return false
                        }

                        return true
                    }

                    return removed
                }

                return Self(worldTransform: worldTransform, ccw: FilterBackGeometry(&_ccw), cw: FilterBackGeometry(&_cw), isInverted: isInverted)

            case .none:
                /// Offset from the split plane to the source's plane.
                let offset = t.inverse.act(coordinate: worldTransform.direct.translation).z

                var isPositive = false

                // If source's polygons are below the split plane:
                if KvIsNegative(offset, alsoIsPositive: &isPositive) {
                    defer { removeAll() }

                    return self
                }
                // The coplanar case:
                else if !isPositive {
                    // Removing coplanar polygons having opposite normals to split plane:
                    switch Math.dot(t.direct.basisZ, worldTransform.direct.basisZ) < 0 {
                    case true:
                        defer { _ccw.removeAll() }

                        return Self(worldTransform: worldTransform, ccw: _ccw, isInverted: isInverted)

                    case false:
                        defer { _cw.removeAll() }

                        return Polygons(worldTransform: worldTransform, cw: _cw, isInverted: isInverted)
                    }
                }
                else { return .init(worldTransform: worldTransform, isInverted: isInverted) }
            }
        }


        /// Deletes (in-place) polygons or it's parts below z = 0 plane defined by given world transformation *t*.
        ///
        /// - SeeAlso: ``drop(below:)``
        @inlinable
        public mutating func clip(by t: Transform) {
            switch worldTransform.localPlaneIntersection(withPlaneFrom: t) {
            case .some(let line):

                func Clip(_ polygons: inout [Polygon2]) {
                    KvArrayKit.mutateAndFilter(&polygons) { polygon in
                        switch polygon.split(by: line) {
                        case (.some(let front), .some):
                            polygon = front
                        case (.some, .none):
                            // Assuming the front part is equal to the polygon.
                            break
                        case (.none, .some), (.none, .none):
                            return false
                        }

                        return true
                    }
                }

                Clip(&_ccw)
                Clip(&_cw)

            case .none:
                /// Offset from the split plane to the source's plane.
                let offset = t.inverse.act(coordinate: worldTransform.direct.translation).z

                var isPositive = false

                // If source's polygons are below the split plane:
                if KvIsNegative(offset, alsoIsPositive: &isPositive) {
                    removeAll()
                }
                // The coplanar case:
                else if !isPositive {
                    // Removing coplanar polygons having opposite normals to split plane:
                    switch Math.dot(t.direct.basisZ, worldTransform.direct.basisZ) < 0 {
                    case true:
                        _ccw.removeAll()
                    case false:
                        _cw.removeAll()
                    }
                }
            }
        }


        // MARK: Operators

        /// - Returns: A transformed deep copy of the receiver.
        @inlinable
        public static func *(lhs: KvAffineTransform3<Math>, rhs: Self) -> Self {
            KvTransform3<Math>(lhs) * rhs
        }


        /// - Returns: A transformed deep copy of the receiver.
        @inlinable
        public static func *(lhs: KvTransform3<Math>, rhs: Self) -> Self {
            let (t, scale) = Transform.from(lhs)

            let (ccw, cw): ([Polygon2], [Polygon2]) = scale.map { scale in

                func Scaled(_ polygons: [Polygon2], by scale: KvAffineTransform3<Math>) -> [Polygon2] {
                    polygons.map { scale * $0 }
                }

                return (Scaled(rhs._ccw, by: scale), Scaled(rhs._cw, by: scale))
            }
            ?? (rhs._ccw.map { $0.clone() }, cw: rhs._cw.map { $0.clone() })

            return Self(worldTransform: t * rhs.worldTransform, ccw: ccw, cw: cw, isInverted: rhs.isInverted)
        }


        // MARK: Auxiliaries

        @usableFromInline
        internal static var flipTransform: Transform {
            let scale = KvTransform3<Math>(scale: [ 1, 1, -1 ])
            return Transform(direct: scale, inverse: scale, plane: [ 0, 0, -1, 0 ])
        }

    }



    // MARK: Cobminimg

    /// Unites in-place the receiver and given object.
    ///
    /// - Warning: Given object *rhs* is changed. Use ``clone()`` to save it.
    @inlinable
    public func formUnion(with rhs: Node) {
        // TODO: Consider simplification to `a.clip(b); b.clip(a); a.insert(b);` resolving overlapping polygons.

        self.clip(by: rhs)
        rhs.clip(by: self)
        rhs.invert()
        rhs.clip(by: self)
        rhs.invert()
        self.insert(rhs)
    }


    /// Subtracts in-place given object from the receiver.
    ///
    /// - Warning: Given object *rhs* is changed. Use ``clone()`` to save it.
    @inlinable
    public func formSubtraction(by rhs: Node) {
        // TODO: Consider simplification by resolving overlapping polygons instead of combining inverted geometry.

        self.invert()
        self.clip(by: rhs)
        rhs.clip(by: self)
        rhs.invert()
        rhs.clip(by: self)
        rhs.invert()
        self.insert(rhs)
        self.invert()
    }


    /// Intersects in-place given object with the receiver.
    ///
    /// - Warning: Given object *rhs* is changed. Use ``clone()`` to save it.
    @inlinable
    public func formIntersection(with rhs: Node) {
        // TODO: Consider simplification by resolving overlapping polygons instead of combining inverted geometry.

        self.invert()
        rhs.clip(by: self)
        rhs.invert()
        self.clip(by: rhs)
        rhs.clip(by: self)
        self.insert(rhs)
        self.invert()
    }


    /// Intersects in-place the receiver with given half space object.
    ///
    /// - Warning: Ensure the half space object contains enough geometry to fill the holes.
    @inlinable
    public func formIntersection(with rhs: KvCsgBspHalfSpace<Math, Vertex, Payload>) {
        // TODO: Implement with no invertions.

        self.invert()
        rhs.clip(by: self)
        self.invert()

        self.clipPolygons(by: rhs.worldTransform * Polygons.flipTransform)

        rhs.traversePolygons(self.insert(_:))
    }



    // MARK: Operations

    @inlinable public var worldTransform: Transform { _polygons.worldTransform }

    /// A boolean value indicating thether the receiver has neigher geometry nor child nodes.
    @inlinable public var isRedundant: Bool { _polygons.isEmpty && _front == nil && _back == nil }


    /// Transforms the receiver.
    @inlinable
    public func apply(_ t: KvTransform3<Math>) {
        _polygons.apply(t)

        _front?.apply(t)
        _back?.apply(t)
    }


    /// Invokes *body* for each polygon of the receiver and all node of the receiver's subtree.
    @inlinable
    public func traversePolygons(_ body: (Polygon3) throws -> Void) rethrows {
        try _polygons.forEach(body)

        try _front?.traversePolygons(body)
        try _back?.traversePolygons(body)
    }


    /// - Returns: A deep copy of the receiver.
    ///
    /// - Note: It's desinged to save objects when combining.
    @inlinable
    public func clone() -> Self {
        Self(polygons: _polygons.clone(), front: _front?.clone(), back: _back?.clone())
    }


    /// Reverses all the geometry in the receiver's subtree.
    @inlinable
    public func invert() {
        _polygons.invert()

        swap(&_front, &_back)

        _front?.invert()
        _back?.invert()
    }


    /// Inserts front child node with given world transformation. If front child node existsts then an exception is thrown.
    ///
    /// - Returns: Created or updated front node.
    ///
    /// - Note: Explicit transformations are used to balance BSP tree and prevent redundant polygon split operations.
    @discardableResult
    public func setFront(_ t: Transform) -> Node {
        switch _front {
        case .none:
            let front = Node(worldTransform: t)
            _front = front
            return front

        case .some(let front):
            if t.plane.isInequal(to: _polygons.worldTransform.plane) {
                let back = front.dropPolygons(below: t)

                front._front = Node(polygons: front._polygons, front: front._front, back: front._back)
                front._back = back
                front._polygons = .init(worldTransform: t)
            }

            return front
        }
    }

    /// Inserts front child node with world transformation defined by given plane. If front child node existsts then an exception is thrown.
    ///
    /// - Returns: Created or updated front node.
    ///
    /// - Note: Explicit transformations are used to balance BSP tree and prevent redundant polygon split operations.
    @discardableResult
    @inlinable
    public func setFront(_ plane: KvPlane3<Math>) -> Node { setFront(Transform(from: plane)) }


    /// Inserts back child node with given world transformation. If back child node existsts then an exception is thrown.
    ///
    /// - Returns: Created or updated back node.
    ///
    /// - Note: Explicit transformations are used to balance BSP tree and prevent redundant polygon split operations.
    @discardableResult
    public func setBack(_ t: Transform) -> Node {
        switch _back {
        case .none:
            let back = Node(worldTransform: t)
            self._back = back
            return back

        case .some(let back):
            if t.plane.isInequal(to: _polygons.worldTransform.plane) {
                let backBack = back.dropPolygons(below: t)

                back._front = Node(polygons: back._polygons, front: back._front, back: back._back)
                back._back = backBack
                back._polygons = .init(worldTransform: t)
            }

            return back
        }
    }

    /// Inserts back child node with world transformation defined by given plane. If back child node existsts then an exception is thrown.
    ///
    /// - Returns: Created or updated back node.
    ///
    /// - Note: Explicit transformations are used to balance BSP tree and prevent redundant polygon split operations.
    @discardableResult
    @inlinable
    public func setBack(_ plane: KvPlane3<Math>) -> Node { setBack(Transform(from: plane)) }


    /// Inserts given polygon into the receiver's subtree.
    public func insert(_ polygon: Polygon3) {
        switch polygon.split(by: worldTransform) {
        case .some(let (frontPolygon, backPolygon)):
            if let polygon = frontPolygon {
                self._front?.insert(polygon) ?? (self._front = .init(with: polygon))
            }
            if let polygon = backPolygon {
                self._back?.insert(polygon) ?? (self._back = .init(with: polygon))
            }

        case .none:
            switch worldTransform.isEqual(to: polygon.worldTransform) {
            case true:
                _polygons.insert(polygon.local)

            case false:
                let t = worldTransform.inverse * polygon.worldTransform.direct

                let vertices = polygon.local.shape.vertices.map {
                    Polygon2.Vertex2(t * $0.underlying)
                }

                _polygons.insert(Polygon2(
                    shape: .init(unsafeVertices: vertices),
                    payload: polygon.payload
                ))
            }
        }
    }


    /// Inserts given polygons into the receiver's subtree.
    @inlinable
    public func insert<Polygons>(_ polygons: Polygons)
    where Polygons : Sequence, Polygons.Element == Polygon3 {
        polygons.forEach(self.insert(_:))
    }


    /// Inserts polygons from a BSP subtree having given root node into the receiver's subtree.
    public func insert(_ bspNode: Node) {
        // TODO: Handle case of equal but non-identical transformations.
        switch worldTransform.isEqual(to: bspNode.worldTransform) {
        case true:
            // TODO: Join neighbour polygons.
            _polygons._ccw.append(contentsOf: bspNode._polygons._ccw)
            _polygons._cw.append(contentsOf: bspNode._polygons._cw)

            switch (_front, bspNode._front) {
            case (.some(let front), .some(let droppedFront)):
                front.insert(droppedFront)
            case (.none, .some(let droppedFront)):
                _front = droppedFront
            case (.some, .none), (.none, .none):
                break
            }

            switch (_back, bspNode._back) {
            case (.some(let back), .some(let droppedBack)):
                back.insert(droppedBack)
            case (.none, .some(let droppedBack)):
                _back = droppedBack
            case (.some, .none), (.none, .none):
                break
            }

        case false:
            bspNode.traversePolygons(self.insert(_:))
        }
    }


    /// Removes the receiver's polygons or their parts inside given BSP tree.
    public func clip<RV, RP>(by rhs: KvCsgBspNode<Math, RV, RP>)
    where RV : KvVertex3Protocol, RV.Math == Math
    {
        // If *rhs* has a back subtree:
        if let rBack = rhs._back {
            // The receiver will contain only front geometry.
            let backPart = dropPolygons(below: rhs.worldTransform)

            if !self.isRedundant, let rFront = rhs._front {
                self.clip(by: rFront)
            }
            // if let b = backPart {
            do {
                backPart.clip(by: rBack)

                // Joining the parts.
                self.insert(backPart)
            }
        }
        else if let rFront = rhs._front {
            // Removing all the geometry to the back of rhs's plane and coplanar polygons having opposite normals.
            clipPolygons(by: rhs.worldTransform)

            clip(by: rFront)
        }
        else if !rhs._polygons.isEmpty {
            // Removing all the geometry to the back of rhs's plane and coplanar polygons having opposite normals.
            clipPolygons(by: rhs.worldTransform)
        }
    }


    /// Splits (in-place) all geometry in the receiver's subtree (including the receiver) by the z = 0 plane defined by given world transformation.
    ///
    /// - Returns: The split result.
    private func dropPolygons(below t: Transform) -> Node {
        Node(polygons: _polygons.drop(below: t),
             front: _front?.dropPolygons(below: t),
             back: _back?.dropPolygons(below: t))
    }


    /// Clips (in-place) all geometry in the receiver's subtree (including the receiver) below the z = 0 plane defined by given world transformation.
    @usableFromInline
    internal func clipPolygons(by t: Transform) {
        _polygons.clip(by: t)

        _front?.clipPolygons(by: t)
        _back?.clipPolygons(by: t)
    }



    // MARK: Operators

    /// - Returns: A copy of the receiver's subtree where all the nodes are transformed.
    @inlinable
    public static func *(lhs: KvAffineTransform3<Math>, rhs: Node) -> Node {
        Node(polygons: lhs * rhs._polygons,
             front: rhs._front.map { lhs * $0 },
             back: rhs._back.map { lhs * $0 })
    }


    /// - Returns: A copy of the receiver's subtree where all the nodes are transformed.
    @inlinable
    public static func *(lhs: KvTransform3<Math>, rhs: Node) -> Node {
        Node(polygons: lhs * rhs._polygons,
             front: rhs._front.map { lhs * $0 },
             back: rhs._back.map { lhs * $0 })
    }

}
