import 'dart:convert';
import 'package:flutter/foundation.dart';
import 'package:shared_preferences/shared_preferences.dart';

class UserProfile {
  final String gender;
  final int? age;

  final String? userPhone;
  final String? emergencyName;
  final String? emergencyPhone;
  final String? doctorName;
  final String? doctorPhone;

  const UserProfile({
    required this.displayName,
    required this.gender,
    this.age,
    this.userPhone,
    this.emergencyName,
    this.emergencyPhone,
    this.doctorName,
    this.doctorPhone,
  });

  UserProfile copyWith({
    String? displayName,
    String? gender,
    int? age,
    String? userPhone,
    String? emergencyName,
    String? emergencyPhone,
    String? doctorName,
    String? doctorPhone,
  }) {
    return UserProfile(
      displayName: displayName ?? this.displayName,
      gender: gender ?? this.gender,
      age: age ?? this.age,
      userPhone: userPhone ?? this.userPhone,
      emergencyName: emergencyName ?? this.emergencyName,
      emergencyPhone: emergencyPhone ?? this.emergencyPhone,
      doctorName: doctorName ?? this.doctorName,
      doctorPhone: doctorPhone ?? this.doctorPhone,
    );
  }

  Map<String, dynamic> toJson() => {
    'display_name': displayName,
    'gender': gender,
    'age': age,
    'user_phone': userPhone,
    'emergency_name': emergencyName,
    'emergency_phone': emergencyPhone,
    'doctor_name': doctorName,
    'doctor_phone': doctorPhone,
  };

  static UserProfile fromJson(Map<String, dynamic> m) => UserProfile(
    displayName: '${m['display_name'] ?? ''}',
    gender: '${m['gender'] ?? 'other'}',
    age: (m['age'] is int)
        ? m['age'] as int
        : int.tryParse('${m['age'] ?? ''}'),
    userPhone: _s(m['user_phone']),
    emergencyName: _s(m['emergency_name']),
    emergencyPhone: _s(m['emergency_phone']),
    doctorName: _s(m['doctor_name']),
    doctorPhone: _s(m['doctor_phone']),
  );

  static String? _s(dynamic v) {
    final t = (v ?? '').toString().trim();
    return t.isEmpty ? null : t;
  }
}

class ProfileStore {
  ProfileStore._();
  static final instance = ProfileStore._();

  final ValueNotifier<UserProfile?> profile = ValueNotifier<UserProfile?>(null);
  static const _k = 'user_profile';

  Future<void> load() async {
    final sp = await SharedPreferences.getInstance();
    final raw = sp.getString(_k);
    if (raw == null) return;
    try {
      profile.value = UserProfile.fromJson(json.decode(raw));
    } catch (_) {}
  }

  Future<void> save(UserProfile p) async {
    final sp = await SharedPreferences.getInstance();
    await sp.setString(_k, json.encode(p.toJson()));
    profile.value = p;
  }

  Future<void> clear() async {
    final sp = await SharedPreferences.getInstance();
    await sp.remove(_k);
    profile.value = null;
  }
}
