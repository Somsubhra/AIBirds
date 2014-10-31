#!/bin/sh

ant compile
ant jar
java -jar ABSoftware.jar -xa -showMBRs
