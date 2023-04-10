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
//  KvConvexPolygonTests.swift
//  kvGeometry
//
//  Created by Svyatoslav Popov on 14.09.2022.
//

import XCTest

@testable import kvGeometry

import simd

import kvKit
import kvTestKit



class KvConvex2Tests : XCTestCase {

    private typealias Vertex<Math : KvMathScope> = KvPosition2<Math, Void>
    private typealias Convex<Math : KvMathScope> = KvConvex2<Vertex<Math>>


    // MARK: : XCTestCase

    override func setUpWithError() throws {
        // Put setup code here. This method is called before the invocation of each test method in the class.
    }


    override func tearDownWithError() throws {
        // Put teardown code here. This method is called after the invocation of each test method in the class.
    }


    // MARK: .TestCase

    private struct TestCase<Math : KvMathScope> {

        typealias Math = Math

        typealias Vertex = KvConvex2Tests.Vertex<Math>
        typealias Convex = KvConvex2Tests.Convex<Math>


        let input: [Vertex]

        let result: ExpectedResult


        // MARK: .Result

        struct ExpectedResult : KvNumericallyEquatable {

            var polygon: [Convex.PolygonVertexIteratorElement]
            var convex: [Convex.VertexIteratorElement]


            func isEqual(to rhs: Self) -> Bool {
                polygon.elementsEqual(rhs.polygon, by: { $0.isEqual(to: $1) })
                && convex.elementsEqual(rhs.convex, by: { $0.isEqual(to: $1) })
            }

        }


        // MARK: Operations

        func run(_ messagePrefix: String) {

            func AssertElementsEqual<Result, Expectation>(
                _ results: Result,
                _ expectations: Expectation,
                _ messagePrefix: String,
                by isEqualBlock: (Result.Element, Expectation.Element) -> Bool
            ) where Result : Sequence, Expectation : Sequence
            {
                var resultIterator = results.makeIterator()
                var expectationIterator = expectations.makeIterator()
                var index = 0

                while true {
                    defer { index += 1}

                    switch (resultIterator.next(), expectationIterator.next()) {
                    case (.some(let result), .some(let expectation)):
                        guard isEqualBlock(result, expectation)
                        else { return XCTFail("\(messagePrefix): index = \(index), result = \(result), expectation = \(expectation)") }
                    case (.some(let result), .none):
                        return XCTFail("\(messagePrefix): there are more results than expected (\(index)). Result at \(index): \(result)")
                    case (.none, .some(let expectation)):
                        return XCTFail("\(messagePrefix): there are less results (\(index)) than expected. Expectation at \(index): \(expectation)")
                    case (.none, .none):
                        return
                    }
                }
            }


            do {
                let polygonResults = IteratorSequence(Convex.VertexIterator.PolygonVertexIterator(input))

                AssertElementsEqual(polygonResults, result.polygon, "\(messagePrefix), polygon vertices and local directions") {
                    $0.isEqual(to: $1)
                }
            }
            do {
                let convexResults = IteratorSequence(Convex.VertexIterator(input))

                AssertElementsEqual(convexResults, result.convex, "\(messagePrefix), convex vertices and direction") {
                    $0.isEqual(to: $1)
                }
            }
        }

    }


    // MARK: Empty Input Test

    func testEmptyInput() throws {

        func MakeTestCase<Math : KvMathScope>(_ math: Math.Type) -> TestCase<Math> {
            TestCase<Math>(
                input: [ ],
                result: .init(polygon: [ ], convex: [ ])
            )
        }

        let prefix = "Empty input"

        MakeTestCase(KvMathFloatScope.self).run(prefix)
        MakeTestCase(KvMathDoubleScope.self).run(prefix)
    }


    // MARK: Single Vertex Input Test

    func testSingleVertexInput() throws {

        func MakeTestCase<Math : KvMathScope>(_ math: Math.Type) -> TestCase<Math> {
            let input: [Vertex<Math>] = [ .zero ]
            let output = Self.verticesAndSteps(from: input)

            return .init(
                input: input,
                result: .init(
                    polygon: output.map { $0.polygonElement(.degenerate) },
                    convex: output.map { $0.convexElement(.invalid) }
                )
            )
        }

        let prefix = "Single vertex"

        MakeTestCase(KvMathFloatScope.self).run(prefix)
        MakeTestCase(KvMathDoubleScope.self).run(prefix)
    }


    // MARK: Repeated Vertex Input Test

    func testRepeatedVertexInput() throws {

        func MakeTestCase<Math : KvMathScope>(_ math: Math.Type) -> TestCase<Math> {
            let input: [Vertex<Math>] = [ .zero, .zero, .zero ]
            let output = Self.verticesAndSteps(from: input).prefix(1)

            return .init(
                input: input,
                result: .init(
                    polygon: output.map { $0.polygonElement(.degenerate) },
                    convex: output.map { $0.convexElement(.invalid) }
                )
            )
        }

        let prefix = "Repeated vertex"

        MakeTestCase(KvMathFloatScope.self).run(prefix)
        MakeTestCase(KvMathDoubleScope.self).run(prefix)
    }


    // MARK: Two Vertices Input Test

    func testTwoVerticesInput1() throws {

        func MakeTestCase<Math : KvMathScope>(_ math: Math.Type) -> TestCase<Math> {
            let input: [Vertex<Math>] = [ .one, .zero ]
            let output = Self.verticesAndSteps(from: input).prefix(1)

            return .init(
                input: input,
                result: .init(
                    polygon: output.map { $0.polygonElement(.degenerate) },
                    convex: output.map { $0.convexElement(.invalid) }
                )
            )
        }

        let prefix = "Two vertices"

        MakeTestCase(KvMathFloatScope.self).run(prefix)
        MakeTestCase(KvMathDoubleScope.self).run(prefix)
    }


    // MARK: Two Vertices then the Last is Repeated Input Test

    func testTwoVerticesInput2() throws {

        func MakeTestCase<Math : KvMathScope>(_ math: Math.Type) -> TestCase<Math> {
            let input: [Vertex<Math>] = [ .one, .zero, .zero, .zero ]
            let output = Self.verticesAndSteps(from: input.prefix(2)).prefix(1)

            return .init(
                input: [ .one, .zero, .zero, .zero ],
                result: .init(
                    polygon: output.map { $0.polygonElement(.degenerate) },
                    convex: output.map { $0.convexElement(.invalid) }
                )
            )
        }

        let prefix = "Two vertices + repeated last"

        MakeTestCase(KvMathFloatScope.self).run(prefix)
        MakeTestCase(KvMathDoubleScope.self).run(prefix)
    }


    // MARK: Two Vertices then the first is Repeated Input Test

    func testTwoVerticesInput3() throws {

        func MakeTestCase<Math : KvMathScope>(_ math: Math.Type) -> TestCase<Math> {
            let input: [Vertex<Math>] = [ .one, .zero, .one, .one, .one ]
            let output = Self.verticesAndSteps(from: input).prefix(1)

            return .init(
                input: [ .one, .zero, .one, .one, .one ],
                result: .init(
                    polygon: output.map { $0.polygonElement(.degenerate) },
                    convex: output.map { $0.convexElement(.invalid) }
                )
            )
        }

        let prefix = "Two vertices + repeated first"

        MakeTestCase(KvMathFloatScope.self).run(prefix)
        MakeTestCase(KvMathDoubleScope.self).run(prefix)
    }


    // MARK: Valid Polygon Test

    func testValidInput() throws {

        func MakeTestCase<Math : KvMathScope>(_ math: Math.Type, isCCW: Bool, count: Int) throws -> TestCase<Math> {
            let start, end: Double
            (start, end) = isCCW ? (0, 2 * .pi) : (2 * .pi, 0)
            let eps: Double = sign(end - start) * 1e-3

            let input = stride(from: start, to: end - eps, by: (end - start) / Double(count))
                .map { Vertex<Math>(x: Math.Scalar(cos($0)), y: Math.Scalar(sin($0))) }

            let output = Self.verticesAndSteps(from: input)

            return TestCase<Math>(
                input: input,
                result: .init(
                    polygon: output.map { $0.polygonElement(isCCW: isCCW) },
                    convex: output.map { $0.convexElement(isCCW: isCCW) }
                )
            )
        }

        try [ 3, 4, 16 ].forEach { count in
            try [ true, false ].forEach { isCCW in
                let prefix = "Valid polygon (\(count), \(isCCW ? "CCW" : "CW"))"

                try MakeTestCase(KvMathFloatScope.self , isCCW: isCCW, count: count).run(prefix)
                try MakeTestCase(KvMathDoubleScope.self, isCCW: isCCW, count: count).run(prefix)
            }
        }
    }


    // MARK: Nonconvex Polygon Test

    func testNonconvexInput() {

        func MakeTestCase<Math : KvMathScope>(_ math: Math.Type, isCCW: Bool, shift: Int) -> TestCase<Math> {
            typealias TC = TestCase<Math>

            var input: [Vertex<Math>] = [
                [  0.0, 0.0 ],
                [  0.5, 1.0 ],
                [  0.0, 0.5 ],
                [ -0.5, 1.0 ],
            ]
            var ccwFlags = [ true, true, false, true ]

            if !isCCW {
                input = Array(KvCollectionKit.cyclicShiftedRight(input.reversed()))
                ccwFlags.indices.forEach { ccwFlags[$0].toggle() }
            }

            input = Array(KvCollectionKit.cyclicShiftedLeft(input, by: shift))
            ccwFlags = Array(KvCollectionKit.cyclicShiftedLeft(ccwFlags, by: shift))

            let output = Self.verticesAndSteps(from: input)
            ccwFlags = Array(KvCollectionKit.cyclicShiftedLeft(ccwFlags))

            var expectedConvex = zip(output, ccwFlags)
                .prefix(1 + ccwFlags.firstIndex(where: { $0 != ccwFlags[0] })!)
                .map { $0.0.convexElement(isCCW: $0.1) }

            expectedConvex[expectedConvex.endIndex - 1].direction = .mixed

            let expected = TC.ExpectedResult(
                polygon: zip(output, ccwFlags).map { $0.0.polygonElement(isCCW: $0.1) },
                convex: expectedConvex
            )

            return .init(input: input, result: expected)
        }

        (0..<4).forEach { shift in
            [ true, false ].forEach { isCCW in
                let prefix = "Nonconvex \(isCCW ? "CCW" : "CW") polygon shifted by \(shift)"

                MakeTestCase(KvMathFloatScope.self , isCCW: isCCW, shift: shift).run(prefix)
                MakeTestCase(KvMathDoubleScope.self, isCCW: isCCW, shift: shift).run(prefix)
            }
        }
    }


    // MARK: Quad with a Backward Vertex Test

    func testBackwardVertex() {

        func MakeTestCase<Math : KvMathScope>(_ math: Math.Type, isCCW: Bool, backIndex: Int) -> TestCase<Math> {
            typealias TC = TestCase<Math>

            var input = Self.quadVertices(Math.self, isCCW: isCCW)

            // Insertion of backward case preserving the direction.
            do {
                let prevIndex = (backIndex > 0 ? backIndex : input.endIndex) - 1
                let v = input[backIndex]
                let offset = 0.1 * (v.coordinate - input[prevIndex].coordinate)

                input[backIndex] -= offset
                input.insert(v, at: backIndex)
            }

            let output = Self.verticesAndSteps(from: input).prefix(backIndex > 0 ? backIndex : input.count)

            let expected: TC.ExpectedResult = {
                switch backIndex > 0 {
                case true:
                    let lastIndex = output.endIndex - 1

                    var expected = TC.ExpectedResult(
                        polygon: output.map { $0.polygonElement(isCCW: isCCW) },
                        convex: output.map { $0.convexElement(isCCW: isCCW) }
                    )

                    expected.polygon[lastIndex].direction = .degenerate
                    expected.convex[lastIndex].direction = .invalid

                    return expected

                case false:
                    let localDirections: [TC.Convex.LocalDirection] = isCCW ? [ .cw, .ccw, .ccw, .ccw, .degenerate ] : [ .ccw, .cw, .cw, .cw, .degenerate ]
                    let directions: [TC.Convex.Direction] = [ isCCW ? .cw : .ccw, .mixed ]

                    return .init(
                        polygon: zip(output, localDirections).map { $0.0.polygonElement($0.1) },
                        convex: zip(output, directions).map { $0.0.convexElement($0.1) }
                    )
                }
            }()

            return .init(input: input, result: expected)
        }

        (0..<4).forEach { backIndex in
            [ true, false ].forEach { isCCW in
                let prefix = "Quad with back index (\(backIndex), \(isCCW ? "CCW" : "CW"))"

                MakeTestCase(KvMathFloatScope.self , isCCW: isCCW, backIndex: backIndex).run(prefix)
                MakeTestCase(KvMathDoubleScope.self, isCCW: isCCW, backIndex: backIndex).run(prefix)
            }
        }
    }


    // MARK: Quad with Duplicated Vertices

    func testDuplicatedInput() {

        func MakeTestCase<Math : KvMathScope>(_ math: Math.Type, isCCW: Bool, count: Int) -> TestCase<Math> {
            let quad = Self.quadVertices(Math.self, isCCW: isCCW)
            let input = Array(KvCollectionKit.cyclicShiftedLeft(
                quad
                    .map { repeatElement($0, count: count) }
                    .joined()
            ))

            let output = Self.verticesAndSteps(from: quad)

            return .init(input: input,
                         result: .init(polygon: output.map { $0.polygonElement(isCCW: isCCW) },
                                       convex: output.map { $0.convexElement(isCCW: isCCW) }))
        }

        [ 2, 3, 64 ].forEach { repeatCount in
            [ true, false ].forEach { isCCW in
                let prefix = "\(isCCW ? "CCW" : "CW") quad with \(repeatCount) copies of each vertex"

                MakeTestCase(KvMathFloatScope.self , isCCW: isCCW, count: repeatCount).run(prefix)
                MakeTestCase(KvMathDoubleScope.self, isCCW: isCCW, count: repeatCount).run(prefix)
            }
        }
    }


    // MARK: Quad with Forward Vertices Test

    func testForwardVertices() {

        func MakeTestCase<Math : KvMathScope>(_ math: Math.Type, isCCW: Bool, count: Int) -> TestCase<Math> {
            let quad = Self.quadVertices(Math.self, isCCW: isCCW)

            let step: Double = 1.0 / Double(1 + count)
            let offsets = stride(from: 0.0, to: 1.0 - 1e-4, by: step)
                .lazy.map { Math.Scalar($0) }

            let input = zip(quad, KvCollectionKit.cyclicShiftedLeft(quad))
                .map { (start, end) in offsets.map { start.mixed(end, t: $0) } }
                .joined()

            let output = Self.verticesAndSteps(from: quad)

            return .init(input: Array(input),
                         result: .init(polygon: output.map { $0.polygonElement(isCCW: isCCW) },
                                       convex: output.map { $0.convexElement(isCCW: isCCW) }))
        }

        [ 1, 2, 3, 64 ].forEach { count in
            [ true, false ].forEach { isCCW in
                let prefix = "\(isCCW ? "CCW" : "CW") quad with \(count) forward vertices on each edge"

                MakeTestCase(KvMathFloatScope.self , isCCW: isCCW, count: count).run(prefix)
                MakeTestCase(KvMathDoubleScope.self, isCCW: isCCW, count: count).run(prefix)
            }
        }
    }


    // MARK: Quad with Forward Case at the End Test

    func testForwardAtTheEnd() {

        func MakeTestCase<Math : KvMathScope>(_ math: Math.Type, isCCW: Bool, startCount: Int, endCount: Int) -> TestCase<Math> {
            let quad = Self.quadVertices(Math.self, isCCW: isCCW)

            let offsetsStart: [Math.Scalar] = {
                switch startCount > 0 {
                case true:
                    let step: Double = 0.5 / Double(startCount)
                    return stride(from: 0.5, to: 1.0 - 1e-4, by: step)
                        .map { Math.Scalar($0) }
                case false:
                    return [ ]
                }
            }()
            let offsetsEnd: [Math.Scalar] = {
                let step: Double = 0.5 / Double(1 + endCount)
                return stride(from: 0.0, to: 0.5 - 1e-4, by: step)
                    .map { Math.Scalar($0) }
            }()

            let v0 = quad.first!, v3 = quad.last!

            let input = [ offsetsStart.map { v3.mixed(v0, t: $0) },
                          quad.dropLast(),
                          offsetsEnd.map { v3.mixed(v0, t: $0) }, ]
                .joined()

            var output = Self.verticesAndSteps(from: quad)
            if startCount > 0 {
                output = Array(KvCollectionKit.cyclicShiftedRight(output))
            }

            return TestCase(input: Array(input),
                            result: .init(polygon: output.map { $0.polygonElement(isCCW: isCCW) },
                                          convex: output.map { $0.convexElement(isCCW: isCCW) }))
        }

        [ 0, 1, 2, 8 ].forEach { count in
            (0...count).forEach { startCount in
                [ true, false ].forEach { isCCW in
                    let endCount = count - startCount
                    let prefix = "\(isCCW ? "CCW" : "CW") quad with \(startCount) forward cases at the start and \(endCount) at the end"

                    MakeTestCase(KvMathFloatScope.self , isCCW: isCCW, startCount: startCount, endCount: endCount).run(prefix)
                    MakeTestCase(KvMathDoubleScope.self, isCCW: isCCW, startCount: startCount, endCount: endCount).run(prefix)
                }
            }
        }
    }


    // MARK: (CCW|CW) Vertices Test

    func testCcwCwVertices() {

        func RunTestCase<Math : KvMathScope>(_ math: Math.Type, isCCW: Bool, reverse isReversed: Bool) {
            let input = Self.quadVertices(Math.self, isCCW: isCCW)

            var output = Self.verticesAndSteps(from: input)
            if !isCCW {
                output.reverse()
            }

            guard let convex = TestCase<Math>.Convex(input, reverse: isReversed)
            else { return XCTFail("Failed to build a \(isCCW ? "CCW" : "CW") quad (reversed: \(isReversed))") }

            XCTAssert(convex.ccwVertices.elementsEqual(output, by: { $0.isEqual(to: $1.vertex) }), "Unexpected CCW vertices from \(isCCW ? "CCW" : "CW") quad (reversed: \(isReversed))")
            XCTAssert(convex.cwVertices.elementsEqual(output.reversed(), by: { $0.isEqual(to: $1.vertex) }), "Unexpected CW vertices from \(isCCW ? "CCW" : "CW") quad (reversed: \(isReversed))")
        }

        [ true, false ].forEach { isCCW in
            [ true, false ].forEach { isReversed in
                RunTestCase(KvMathFloatScope.self , isCCW: isCCW, reverse: false)
                RunTestCase(KvMathDoubleScope.self, isCCW: isCCW, reverse: true )
            }
        }
    }


    // MARK: isEqual Test

    func testIsEqual() {

        func RunTestCase<Math : KvMathScope>(_ math: Math.Type, isCCW: Bool) {
            let input = Self.quadVertices(Math.self, isCCW: isCCW)

            guard let lhs = TestCase<Math>.Convex(input)
            else { return XCTFail("Failed to build a \(isCCW ? "CCW" : "CW") quad") }

            [ 0 as Int, 1, 2, 3 ].forEach { shift in
                [ false, true ].forEach { isReversed in
                    let input2 = KvCollectionKit.cyclicShiftedLeft(input, by: shift)
                    guard let rhs = (!isReversed
                                     ? TestCase<Math>.Convex(input2)
                                     : TestCase<Math>.Convex(input2.reversed(), reverse: true))
                    else { return XCTFail("Failed to build a \(isCCW ? "CCW" : "CW") quad: shift=\(shift), reverse=\(isReversed)") }

                    XCTAssert(lhs.isEqual(to: rhs), "Two equal quads: \(isCCW ? "CCW" : "CW"), shift=\(shift), reverse=\(isReversed)")
                }
            }
        }

        [ true, false ].forEach { isCCW in
            RunTestCase(KvMathFloatScope.self , isCCW: isCCW)
            RunTestCase(KvMathDoubleScope.self, isCCW: isCCW)
        }
    }


    // MARK: Split Test

    func testSplit() {

        func RunSplit<Math : KvMathScope>(_ math: Math.Type, isCCW: Bool) {
            let line: KvLine2<Math> = [ 0, 1, -0.5 ]

            let input = Self.quadVertices(Math.self, isCCW: isCCW)
            let (front, back): ([Vertex<Math>], [Vertex<Math>]) = isCCW
            ? ([ [ 0, 0.5 ], [ 1, 0.5 ], [ 1, 1 ], [ 0, 1 ] ],
               [ [ 0, 0 ], [ 1, 0 ], [ 1, 0.5 ], [ 0, 0.5 ] ])
            : ([ [ 0, 0.5 ], [ 0, 1 ], [ 1, 1 ], [ 1, 0.5 ] ],
               [ [ 0, 0 ], [ 0, 0.5 ], [ 1, 0.5 ], [ 1, 0 ] ])

            guard let quad = TestCase<Math>.Convex(input)
            else { return XCTFail("Failed to build a \(isCCW ? "CCW" : "CW") quad") }

            guard let quadFront = TestCase<Math>.Convex(front)
            else { return XCTFail("Failed to build a \(isCCW ? "CCW" : "CW") front quad") }
            guard let quadBack = TestCase<Math>.Convex(back)
            else { return XCTFail("Failed to build a \(isCCW ? "CCW" : "CW") back quad") }

            let result = quad.split(by: line)

            XCTAssert(result.front?.isEqual(to: quadFront) == true)
            XCTAssert(result.back?.isEqual(to: quadBack) == true)

            XCTAssert(result.front?.isCCW == isCCW)
            XCTAssert(result.back?.isCCW == isCCW)
        }


        func RunNoSplit<Math : KvMathScope>(_ math: Math.Type, isCCW: Bool) {
            do {
                let input = Self.quadVertices(Math.self, isCCW: isCCW)

                guard let quad = TestCase<Math>.Convex(input)
                else { return XCTFail("Failed to build a \(isCCW ? "CCW" : "CW") quad") }

                typealias Line = KvLine2<Math>

                let testCases: [(line: Line, isFront: Bool)] = [
                    ([  0,  1,  1 ] as Line,  true), ([  0,  1, -2 ] as Line, false),
                    ([  0, -1, -1 ] as Line, false), ([  0, -1,  2 ] as Line,  true),
                    ([  1,  0,  1 ] as Line,  true), ([  1,  0, -2 ] as Line, false),
                    ([ -1,  0, -1 ] as Line, false), ([ -1,  0,  2 ] as Line,  true),

                    ([  0,  1, 0 ] as Line,  true), ([  0,  1, -1 ] as Line, false),
                    ([  0, -1, 0 ] as Line, false), ([  0, -1,  1 ] as Line,  true),
                    ([  1,  0, 0 ] as Line,  true), ([  1,  0, -1 ] as Line, false),
                    ([ -1,  0, 0 ] as Line, false), ([ -1,  0,  1 ] as Line,  true),

                    (Line(in: Math.normalize([  1, -1 ]), at: [ 0, 0 ]),  true),
                    (Line(in: Math.normalize([ -1,  1 ]), at: [ 0, 0 ]), false),
                    (Line(in: Math.normalize([  1, -1 ]), at: [ 1, 1 ]), false),
                    (Line(in: Math.normalize([ -1,  1 ]), at: [ 1, 1 ]),  true),

                    (Line(in: Math.normalize([  1,  1 ]), at: [ 1, 0 ]),  true),
                    (Line(in: Math.normalize([ -1, -1 ]), at: [ 1, 0 ]), false),
                    (Line(in: Math.normalize([  1,  1 ]), at: [ 0, 1 ]), false),
                    (Line(in: Math.normalize([ -1, -1 ]), at: [ 0, 1 ]),  true),
                ]

                testCases.forEach { (line, isFront) in
                    let result = quad.split(by: line)

                    switch isFront {
                    case true:
                        XCTAssert(result.front != nil)
                        XCTAssert(result.back == nil)

                        XCTAssert(result.front?.isCCW == isCCW)

                    case false:
                        XCTAssert(result.front == nil)
                        XCTAssert(result.back != nil)

                        XCTAssert(result.back?.isCCW == isCCW)
                    }
                }
            }
            // A real case where line contains one of vertices and all oter vertices are in positive half space.
            do {
                let input: [KvPosition2<Math, Void>] = [
                    [ 1.7260361466778038, 0.0019036039196929089 ],
                    [ 1.7832921656971408, 1.6724486737107114 ],
                    [ 0.0, 0.0 ],
                    [ 0.6859084618411143, -1.655929511123291 ],
                ]
                let line: KvLine2<Math> = [ 0.6054502632390406, 0.7958831438997667, 0.902642926609616 ]

                guard let convex = KvConvex2(input, reverse: !isCCW)
                else { return XCTFail("Failed to build a \(isCCW ? "CCW" : "CW") convex polygon") }

                let result = convex.split(by: line)

                KvAssertEqual(result.front, convex)
                XCTAssertNil(result.back)
            }
        }


        func Run<Math : KvMathScope>(_ math: Math.Type, isCCW: Bool) {
            RunSplit(Math.self, isCCW: isCCW)
            RunNoSplit(Math.self, isCCW: isCCW)
        }


        [ true, false ].forEach { isCCW in
            Run(KvMathFloatScope.self , isCCW: isCCW)
            Run(KvMathDoubleScope.self, isCCW: isCCW)
        }
    }


    // MARK: .VertexAndStep

    private struct VertexAndStep<Math : KvMathScope> : Hashable {

        typealias Convex = KvConvex2Tests.Convex<Math>

        typealias ConvexElement = Convex.VertexIteratorElement
        typealias PolygonElement = Convex.PolygonVertexIteratorElement


        var vertex: Convex.Vertex
        var step: Convex.PolygonVertexIteratorElement.Step


        // MARK: Operations

        func polygonElement(_ direction: Convex.LocalDirection) -> PolygonElement {
            .init(vertex: vertex, step: step, direction: direction)
        }

        func polygonElement(isCCW: Bool) -> PolygonElement {
            .init(vertex: vertex, step: step, direction: isCCW ? .ccw : .cw)
        }


        func convexElement(_ direction: Convex.Direction) -> ConvexElement {
            .init(vertex: vertex, step: step, direction: direction)
        }

        func convexElement(isCCW: Bool) -> ConvexElement {
            .init(vertex: vertex, step: step, direction: isCCW ? .ccw : .cw)
        }

    }


    // MARK: Auxiliaries

    private static func quadVertices<Math : KvMathScope>(_ math: Math.Type, isCCW: Bool) -> [Vertex<Math>] {
        return (isCCW
                ? [ [ 0, 0 ], [ 1, 0 ], [ 1, 1 ], [ 0, 1 ] ]
                : [ [ 0, 0 ], [ 0, 1 ], [ 1, 1 ], [ 1, 0 ] ])
    }


    private static func verticesAndSteps<Vertices, Math>(from input: Vertices) -> [VertexAndStep<Math>]
    where Math : KvMathScope, Vertices : Collection, Vertices.Element == Vertex<Math>
    {
        zip(KvCollectionKit.cyclicShiftedLeft(input, by: 1), KvCollectionKit.cyclicShiftedLeft(input, by: 2))
            .map { .init(vertex: $0.0, step: .init(from: $0.0.coordinate, to: $0.1.coordinate)) }
    }

}
