import 'dart:async';
import 'dart:convert';
import 'package:flutter_reactive_ble/flutter_reactive_ble.dart';

class SolirisBle {
  static final Uuid svcCtrl = Uuid.parse(
    "c0de0001-2bad-4b0b-a3f8-9b3b5f2a0001",
  );
  static final Uuid chrCtrl = Uuid.parse(
    "c0de0002-2bad-4b0b-a3f8-9b3b5f2a0001",
  );

  final _ble = FlutterReactiveBle();

  DiscoveredDevice? _device;
  StreamSubscription<DiscoveredDevice>? _scanSub;
  StreamSubscription<ConnectionStateUpdate>? _connSub;
  QualifiedCharacteristic? _ctrlChar;

  bool get isConnected => _ctrlChar != null;

  Future<void> connect({String namePrefix = "Soliris"}) async {
    final c = Completer<void>();
    await _scanSub?.cancel();
    _scanSub = _ble
        .scanForDevices(withServices: [], scanMode: ScanMode.balanced)
        .listen(
          (d) {
            if (d.name.startsWith(namePrefix)) {
              _device = d;
              _scanSub?.cancel();
              _connSub = _ble
                  .connectToDevice(id: d.id)
                  .listen(
                    (u) async {
                      if (u.connectionState ==
                          DeviceConnectionState.connected) {
                        _ctrlChar = QualifiedCharacteristic(
                          deviceId: d.id,
                          serviceId: svcCtrl,
                          characteristicId: chrCtrl,
                        );
                        c.complete();
                      }
                    },
                    onError: (e) {
                      if (!c.isCompleted) c.completeError(e);
                    },
                  );
            }
          },
          onError: (e) {
            if (!c.isCompleted) c.completeError(e);
          },
        );

    return c.future.timeout(const Duration(seconds: 20));
  }

  Future<void> disconnect() async {
    await _scanSub?.cancel();
    await _connSub?.cancel();
    _scanSub = null;
    _connSub = null;
    _ctrlChar = null;
    _device = null;
  }

  Future<void> _writeJson(Map<String, dynamic> m) async {
    if (_ctrlChar == null) throw StateError("BLE not connected");
    final payload = utf8.encode(jsonEncode(m));
    await _ble.writeCharacteristicWithResponse(_ctrlChar!, value: payload);
  }

  Future<void> sendCtrl(Map<String, dynamic> m) => _writeJson(m);
  Future<void> setLed(bool on) => _writeJson({"led": on});
  Future<void> setBuzz(bool on) => _writeJson({"buzz": on});
  Future<void> play(String k) => _writeJson({"play": k});
}

final solirisBle = SolirisBle();
