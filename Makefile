.PHONY unittest:
unittest:
	./scripts/run_tests.sh
.PHONY test:
test:
	pipenv run py.test
.PHONY test:
test-with-artifact:
	pipenv run py.test --junit-xml=test-reports/junit.xml
