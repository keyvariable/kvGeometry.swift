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
//  KvBConvex2Tests.swift
//  kvGeometry
//
//  Created by Svyatoslav Popov on 28.09.2022.
//

import XCTest

@testable import kvGeometry

import kvKit
import kvTestKit



class KvBConvex2Tests : XCTestCase {

    // MARK: : XCTestCase

    override func setUpWithError() throws {
        // Put setup code here. This method is called before the invocation of each test method in the class.
    }


    override func tearDownWithError() throws {
        // Put teardown code here. This method is called after the invocation of each test method in the class.
    }



    // MARK: Horizontal Segment Test

    func testSegmentAtY() throws {

        func Run<Math : KvMathScope>(_ math: Math.Type) throws {
            let bconvex = try makeTriangleBConvex(math)

            let testCases: [Math.Scalar : ClosedRange<Math.Scalar>?] = [
                -1.0 : nil,
                -0.001 : nil,
                 0.0 : 0...1,
                 0.1 : 0.05...0.95,
                 0.5 : 0.25...0.75,
                 0.9 : 0.45...0.55,
                 1.0 : 0.5...0.5,
                 1.1 : nil,
            ]

            for (y, expected) in testCases {
                KvAssertEqual(bconvex.segment(y: y), expected, by: KvIs(_:equalTo:))
            }
        }

        try Run(KvMathFloatScope.self)
        try Run(KvMathDoubleScope.self)
    }



    // MARK: Auxiliaries

    private func makeTriangleBConvex<Math : KvMathScope>(_ math: Math.Type) throws -> KvBConvex2<Math> {
        guard let bconvex = KvBConvex2<Math>(triangle: [ 0, 0 ], [ 1, 0 ], [ 0.5, 1 ])
        else { throw KvError("Failed to initialize a valid BConvex for a triangle") }

        return bconvex
    }

}
