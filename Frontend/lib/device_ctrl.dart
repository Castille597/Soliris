import 'dart:convert';
import 'package:http/http.dart' as http;

import 'soliris_ble.dart';

const String backendBase = 'http://192.168.1.26:5050';

const String? deviceIp = null;

Future<void> deviceCtrl(Map<String, dynamic> j) async {
  try {
    if (solirisBle.isConnected) {
      await solirisBle.sendCtrl(j);
      return;
    }
  } catch (_) {}

  if (backendBase.isNotEmpty) {
    final uri = Uri.parse(
      deviceIp == null
          ? '$backendBase/device/ctrl'
          : '$backendBase/device/ctrl?ip=$deviceIp',
    );
    final res = await http.post(
      uri,
      headers: {'Content-Type': 'application/json'},
      body: json.encode(j),
    );
    if (res.statusCode >= 300) {
      throw Exception('HTTP ${res.statusCode}: ${res.body}');
    }
    return;
  }

  if (deviceIp != null) {
    final uri = Uri.parse('$deviceIp/ctrl');
    final res = await http.post(
      uri,
      headers: {'Content-Type': 'application/json'},
      body: json.encode(j),
    );
    if (res.statusCode >= 300) {
      throw Exception('HTTP ${res.statusCode}: ${res.body}');
    }
    return;
  }

  throw StateError(
    'No route to device (BLE not connected, no backend/deviceIp configured).',
  );
}
