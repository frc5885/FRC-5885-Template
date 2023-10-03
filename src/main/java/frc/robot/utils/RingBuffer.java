package frc.robot.utils;

public class RingBuffer {

  private final Double[] buffer;
  private int head;
  private int tail;

  public RingBuffer(int size) {
    buffer = new Double[size];
    head = 0;
    tail = 0;
  }

  public void add(Double obj) {
    buffer[head] = obj;
    head = (head + 1) % buffer.length;
    if (head == tail) {
      tail = (tail + 1) % buffer.length;
    }
  }

  public Double get(int index) {
    return buffer[(tail + index) % buffer.length];
  }

  public int size() {
    return head - tail;
  }

  public boolean isEmpty() {
    return head == tail;
  }

  public void clear() {
    head = 0;
    tail = 0;
  }

  public Double[] toArray() {
    Double[] result = new Double[size()];
    for (int i = 0; i < result.length; i++) {
      result[i] = get(i);
    }
    return result;
  }

  public double getAverage() {
    double sum = 0;
    for (int i = 0; i < size(); i++) {
      sum += (double) get(i);
    }
    return sum / size();
  }
}
