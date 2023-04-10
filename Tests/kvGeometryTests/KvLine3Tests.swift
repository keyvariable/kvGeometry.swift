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
//  KvLine3Tests.swift
//  kvGeometry
//
//  Created by Svyatoslav Popov on 28.09.2022.
//

import XCTest

@testable import kvGeometry

import kvKit



class KvLine3Tests : XCTestCase {

    typealias L<Math : KvMathScope> = KvLine3<Math>



    // MARK: : XCTestCase

    override func setUpWithError() throws {
        // Put setup code here. This method is called before the invocation of each test method in the class.
    }


    override func tearDownWithError() throws {
        // Put teardown code here. This method is called after the invocation of each test method in the class.
    }



    // MARK: .contains Coordiante Tests

    func testContainsCoordinate() {

        func Run<Math : KvMathScope>(_ math: Math.Type)
        where Math.Scalar.RawSignificand : FixedWidthInteger
        {
            (0..<50).forEach { _ in
                let line = L<Math>(in: Math.randomNonzero3(in: -10...10),
                                   at: Math.random3(in: -100...100))

                var t: Math.Scalar = -100
                while t <= 100 {
                    defer { t += 25 }

                    let c_in = line.origin + t * line.front
                    XCTAssert(line.contains(c_in), "contains: line = (in: \(line.front), at: \(line.origin)), c = \(c_in)")

                    let c_out = c_in + 1e-3 * Math.Quaternion(from: .unitZ, to: line.front).act(.unitY)
                    XCTAssert(!line.contains(c_out), "!contains: line = (in: \(line.front), at: \(line.origin)), c = \(c_out)")
                }
            }
        }

        Run(KvMathFloatScope.self)
        Run(KvMathDoubleScope.self)
    }



    // MARK: Auxiliaries

    func assertEqual<Math>(_ line: L<Math>, _ expected: L<Math>) {
        XCTAssert(line.isEqual(to: expected), "Resulting \(line) line is not equal to expected \(expected) line")
    }

}
