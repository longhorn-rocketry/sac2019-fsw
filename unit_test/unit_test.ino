#include <math.h>
#include <photonic.h>
#include <Wire.h>

using namespace photonic;

void setup() {
  out("Running history tests...\n");
  float start = millis() / 1000.0;
  history<float> hist(3);
  test("hs0", hist.at_capacity(), false);
  hist.add(3);
  test("hs1", hist.at_capacity(), false);
  test("hs2", hist[0], 3);
  hist.add(4);
  hist.add(5);
  test("hs3", hist.at_capacity(), true);
  test("hs4", hist.mean(), 4);
  test("hs5", hist.stdev(), 0.816);
  hist.add(6);
  test("hs6", hist.at_capacity(), true);
  test("hs7", hist[2], 6);
  test("hs8", hist[0], 4);
  float end = millis() / 1000.0;
  Serial.printf("%f s elapsed\n", end - start);


  out("Running heap tests...\n");
  start = millis() / 1000.0;
  TelemetryHeap heap(0, 2048, new ArduinoHeapIO());
  int id0 = heap.add_block(4);
  int id1 = heap.add_block(2);
  heap.logc(id0, 9.807);
  heap.logc(id0, -3.141);
  heap.logc(id1, 2.718);
  float *arr0 = heap.decompress(0, 4);
  float *arr1 = heap.decompress(4, 6);
  test("hp0", arr0[0], 9.807);
  test("hp1", arr0[1], -3.141);
  test("hp2", arr1[0], 2.718);
  end = millis() / 1000.0;
  Serial.printf("%f s elapsed\n", end - start);
}

void loop() {}

void test(const char tag[], float actual, float expected) {
  out(tag);
  out(": ");
  Serial.printf("%f == %f; ", actual, expected);
  out(approx(actual, expected) ? "SUCCESS" : "FAILED");
  out("\n");
}

void test(const char tag[], bool actual, bool expected) {
  out(tag);
  out(": ");
  Serial.printf("%d == %d; ", (int)actual, (int)expected);
  out(actual == expected ? "SUCCESS" : "FAILED");
  out("\n");
}

void out(const char str[]) {
  Serial.println(str);
}

bool approx(float a, float b) {
  return fabs(a - b) < 0.01;
}
