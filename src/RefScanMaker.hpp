#pragma once

class RefScanMaker {

public:
  RefScanMaker();
  ~RefScanMaker();

  void makeRefScan();
  void getRefScan(Scan2D &refScan);

private:
  Scan2D refScan;

};