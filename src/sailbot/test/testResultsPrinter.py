import xml.etree.ElementTree as ET


def parseXML(xmlfile):
    # create element tree object
    tree = ET.parse(xmlfile)
    root = tree.getroot()

    suites = root.findall("testsuite")

    for suite in suites:
        tests = suite.findall("testcase")
        for test in tests:
            for child in test:
                if child.tag == "failure" or child.tag == "error":
                    print(f"{child.tag} : {child.text}")

        suiteVals = dict(suite.items())
        print(
            f"{suiteVals['name']}: (errors: {suiteVals['errors']}, failures: {suiteVals['failures']}, skipped {suiteVals['skipped']}, tests: {suiteVals['tests']})"
        )


parseXML("build/sailbot/pytest.xml")
