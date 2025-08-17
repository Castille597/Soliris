import 'dart:convert';
import 'package:flutter/material.dart';
import '../theme/scale_utils.dart';

class RecommendationCard extends StatelessWidget {
  const RecommendationCard({
    super.key,
    required this.msg,
    this.title = 'Wellness recommendations',
    this.isLive = true,
    this.margin = const EdgeInsets.symmetric(horizontal: 12, vertical: 12),
    this.showInsights = true,
    this.showChips = true,
    this.showMetrics = true,
  });

  final Map<String, dynamic> msg;
  final String title;
  final bool isLive;
  final EdgeInsetsGeometry margin;

  final bool showInsights;
  final bool showChips;
  final bool showMetrics;

  Map<String, dynamic> _asMap(dynamic v) {
    if (v is Map) return v.map((k, val) => MapEntry(k.toString(), val));
    if (v is String) {
      try {
        final d = jsonDecode(v);
        if (d is Map) return d.map((k, val) => MapEntry(k.toString(), val));
      } catch (_) {}
    }
    return const {};
  }

  List<String> _asStringList(dynamic v) {
    if (v is List) {
      return v.map((e) => '$e').where((s) => s.trim().isNotEmpty).toList();
    }
    if (v is String) {
      try {
        final d = jsonDecode(v);
        if (d is List) {
          return d.map((e) => '$e').where((s) => s.trim().isNotEmpty).toList();
        }
      } catch (_) {}
      final s = v.trim();
      return s.isEmpty ? const [] : <String>[s];
    }
    return const [];
  }

  num? _num(dynamic v) {
    if (v is num) return v;
    if (v is String) return num.tryParse(v);
    return null;
  }

  String _str(dynamic v) => v == null ? '' : v.toString();

  int? _epochMs(dynamic ts) {
    final n = _num(ts);
    if (n == null) return null;
    final i = n.toInt();
    return (i < 1000000000000) ? i * 1000 : i;
  }

  String _stripDiacritics(String s) {
    const from = 'áàâäãåÁÀÂÄÃÅéèêëÉÈÊËíìîïÍÌÎÏóòôöõÓÒÔÖÕúùûüÚÙÛÜçÇñÑ';
    const to = 'aaaaaaAAAAAAeeeeEEEEiiiiIIIIoooooOOOOOuuuuUUUUcCnN';
    final a = from.runes.toList(), b = to.runes.toList();
    final buf = StringBuffer();
    for (final ch in s.runes) {
      final i = a.indexOf(ch);
      buf.writeCharCode(i >= 0 ? b[i] : ch);
    }
    return buf.toString();
  }

  String _extractUserName(Map<String, dynamic> m) {
    final v = m['userId'] ?? m['user'] ?? m['uid'] ?? m['name'];
    return _str(v);
  }

  String _toSecondPerson(String s, {String? name}) {
    if (s.isEmpty) return s;
    var out = s;
    if (name != null && name.isNotEmpty) {
      final n1 = RegExp.escape(name);
      final n2 = RegExp.escape(_stripDiacritics(name));
      out = out.replaceAll(
        RegExp(r"\b($n1|$n2)'s\b", caseSensitive: false),
        "your",
      );
      out = out.replaceAll(
        RegExp(r"\b($n1|$n2)\b", caseSensitive: false),
        "you",
      );
    }
    final repl = <RegExp, String>{
      RegExp(r"\b(the\s+)?(user|patient|wearer)\b", caseSensitive: false):
          "you",
      RegExp(r"\b(she|he|they)\b", caseSensitive: false): "you",
      RegExp(r"\b(her|his|their)\b", caseSensitive: false): "your",
      RegExp(r"\b(herself|himself|themselves)\b", caseSensitive: false):
          "yourself",
      RegExp(r"\b(she|he|they)'s\b", caseSensitive: false): "you're",
    };
    repl.forEach((re, rep) => out = out.replaceAll(re, rep));
    out = out.replaceAll(
      RegExp(r"\bcheck on (you|yourself)\b", caseSensitive: false),
      "check in with yourself",
    );
    return out.replaceAll(RegExp(r"\s{2,}"), " ").trim();
  }

  String _motivationalFallback(Map<String, dynamic> msg) {
    final now = DateTime.now();
    final tele = _asMap(msg['telemetry']);
    final tsMs = _epochMs(tele['ts'] ?? msg['ts']);
    int? minutesSince;
    if (tsMs != null) {
      final last = DateTime.fromMillisecondsSinceEpoch(tsMs, isUtc: false);
      minutesSince = now.difference(last).inMinutes;
    }

    final ctx = _asMap(msg['context']);
    final temp = _num(ctx['ambient_temp'])?.toDouble();
    final uv = _num(ctx['uv_index'])?.toDouble();
    final aqi = _num(ctx['aqi'])?.toDouble();

    if (minutesSince != null && minutesSince > 20) {
      return "No recent measurements. Please check the device is worn and connected. Meanwhile, take a sip of water 💧.";
    }
    if (temp != null && temp >= 30)
      return "Warm day. Stay cool and sip water regularly 💧.";
    if (uv != null && uv >= 6)
      return "UV is high. Prefer shade if you go outside, and keep hydrated.";
    if (aqi != null && aqi >= 80)
      return "Air quality is average today. Avoid exertion outdoors and ventilate at calmer hours.";

    final h = now.hour;
    if (h >= 6 && h < 11)
      return "Easy start: a glass of water and a gentle stretch ✨.";
    if (h >= 11 && h < 17)
      return "All clear ✅. Take three deep breaths… and a sip of water.";
    if (h >= 17 && h < 22)
      return "Nice afternoon. Relax your shoulders and hydrate 💛.";
    return "Quiet evening. Hydrate and get some rest—you’ve earned it 😴.";
  }

  List<String> _whyNow(Map<String, dynamic> tele, Map<String, dynamic> ctx) {
    final out = <String>[];

    final hr = _num(tele['hr']);
    final spo2 = _num(tele['spo2']);
    final skin = _num(tele['temp_skin']);
    final co2 = _num(tele['co2']) ?? _num(tele['co2_ppm']);
    final steps = _num(tele['steps']);

    final amb = _num(ctx['ambient_temp']);
    final aqi = _num(ctx['aqi']);
    final uv = _num(ctx['uv_index']);
    final city = _str(ctx['city']);

    if (hr != null) {
      if (hr >= 120)
        out.add("Your heart rate is elevated at ${hr.toInt()} bpm.");
      else if (hr <= 50)
        out.add("Your heart rate is on the low side at ${hr.toInt()} bpm.");
    }
    if (spo2 != null && spo2 <= 92) {
      out.add("Your oxygen level is low at ${spo2.toInt()} %.");
    }
    if (skin != null && skin >= 37.8) {
      out.add(
        "Your skin temperature is high at ${skin.toStringAsFixed(1)} °C.",
      );
    }

    if (amb != null) {
      final where = city.isEmpty ? "" : " in $city";
      if (amb >= 35)
        out.add("It’s very hot at ${amb.toStringAsFixed(1)} °C$where.");
      else if (amb >= 30)
        out.add("It’s warm at ${amb.toStringAsFixed(1)} °C$where.");
      else if (amb <= 5)
        out.add("It’s cold at ${amb.toStringAsFixed(1)} °C$where.");
    }
    if (aqi != null)
      out.add("Outdoor air is ${_aqiLabel(aqi)} (AQI ${aqi.toInt()}).");
    if (uv != null && uv >= 3)
      out.add("UV index is ${uv.toString()} (${_uvLabel(uv)}).");
    if (co2 != null && co2 >= 1000)
      out.add("Indoor air feels ${_co2Label(co2)} (CO₂ ${co2.toInt()} ppm).");
    if (steps != null && steps == 0)
      out.add(
        "No recent steps detected—take a short break and check the fit of your device.",
      );

    return out;
  }

  String _aqiLabel(num aqi) {
    if (aqi >= 301) return "hazardous";
    if (aqi >= 201) return "very unhealthy";
    if (aqi >= 151) return "unhealthy";
    if (aqi >= 101) return "unhealthy for sensitive groups";
    if (aqi >= 51) return "moderate";
    return "good";
  }

  String _uvLabel(num uv) {
    if (uv >= 11) return "extreme";
    if (uv >= 8) return "very high";
    if (uv >= 6) return "high";
    if (uv >= 3) return "moderate";
    return "low";
  }

  String _co2Label(num co2) {
    if (co2 >= 2000) return "poor";
    if (co2 >= 1200) return "stuffy";
    return "fresh";
  }

  Color _blend(Color base, Color tint, double t) =>
      Color.alphaBlend(tint.withOpacity(t), base);

  ({Color bg, Color fg, Color outline, Color dot}) _colorsForRisk(
    String risk,
    ThemeData theme,
  ) {
    final cs = theme.colorScheme;
    final base = theme.cardColor;
    final isDark = theme.brightness == Brightness.dark;

    switch (risk) {
      case 'high':
        return (
          bg: cs.errorContainer,
          fg: cs.onErrorContainer,
          outline: cs.error.withOpacity(isDark ? .45 : .35),
          dot: cs.error,
        );
      case 'medium':
      case 'moderate':
        final tint = cs.primary;
        return (
          bg: _blend(base, tint, isDark ? .14 : .10),
          fg: theme.colorScheme.onSurface,
          outline: tint.withOpacity(isDark ? .35 : .25),
          dot: tint,
        );
      default:
        final tint = cs.secondary;
        return (
          bg: _blend(base, tint, isDark ? .10 : .08),
          fg: theme.colorScheme.onSurface,
          outline: tint.withOpacity(isDark ? .30 : .22),
          dot: cs.primary,
        );
    }
  }

  Widget _chip(
    BuildContext context,
    IconData icon,
    String label,
    Color fg,
    Color outline,
  ) {
    final theme = Theme.of(context);
    final double hp = 10.sx(context);
    final double vp = 6.sx(context);
    return Container(
      padding: EdgeInsets.symmetric(horizontal: hp, vertical: vp),
      margin: EdgeInsets.only(right: 8.sx(context), bottom: 8.sx(context)),
      decoration: BoxDecoration(
        color: fg.withOpacity(0.06),
        borderRadius: BorderRadius.circular(999),
        border: Border.all(color: outline),
      ),
      child: Row(
        mainAxisSize: MainAxisSize.min,
        children: [
          Icon(icon, size: 14, color: fg.withOpacity(.8)),
          SizedBox(width: 6.sx(context)),
          Text(label, style: theme.textTheme.bodySmall?.copyWith(color: fg)),
        ],
      ),
    );
  }

  @override
  Widget build(BuildContext context) {
    final theme = Theme.of(context);

    final rec = _asMap(msg['recommendation']);
    final ctx = _asMap(msg['context']);
    final tele = _asMap(msg['telemetry']);

    final riskRaw = _str(rec['risk']).toLowerCase().trim();
    final risk = riskRaw.isEmpty ? 'low' : riskRaw;

    final colors = _colorsForRisk(risk, theme);

    final double pad = 16.sx(context);
    final double gap = 8.sx(context);
    final double radius = 16.sx(context);
    final double dot = 10.sx(context);
    final double pillHP = 8.sx(context);
    final double pillVP = 2.sx(context);

    final name = _extractUserName(msg);
    final head = _toSecondPerson(_str(rec['headline']).trim(), name: name);
    final expl = _toSecondPerson(_str(rec['explanation']).trim(), name: name);
    final acts = _asStringList(
      rec['actions'],
    ).map((a) => _toSecondPerson(a, name: name)).toList();

    final tags = _asStringList(rec['tags']);
    final city = _str(ctx['city']).trim();

    final amb = _num(ctx['ambient_temp']);
    final aqi = _num(ctx['aqi']);
    final uv = _num(ctx['uv_index']);
    final hr = _num(tele['hr']);
    final spo2 = _num(tele['spo2']);
    final skin = _num(tele['temp_skin']);
    final co2 = _num(tele['co2']) ?? _num(tele['co2_ppm']);

    final why = showInsights
        ? _whyNow(tele, ctx).map((s) => _toSecondPerson(s, name: name)).toList()
        : const <String>[];

    return Card(
      elevation: 0,
      color: colors.bg,
      margin: margin,
      shape: RoundedRectangleBorder(
        borderRadius: BorderRadius.circular(radius),
        side: BorderSide(color: colors.outline, width: 1),
      ),
      child: Padding(
        padding: EdgeInsets.all(pad),
        child: Column(
          crossAxisAlignment: CrossAxisAlignment.start,
          children: [
            Row(
              crossAxisAlignment: CrossAxisAlignment.start,
              children: [
                Container(
                  width: dot,
                  height: dot,
                  margin: EdgeInsets.only(top: 6.sx(context)),
                  decoration: BoxDecoration(
                    color: colors.dot,
                    shape: BoxShape.circle,
                  ),
                ),
                SizedBox(width: gap),
                Expanded(
                  child: Text(
                    title,
                    softWrap: true,
                    style: theme.textTheme.titleMedium?.copyWith(
                      fontWeight: FontWeight.w700,
                      color: colors.fg,
                    ),
                  ),
                ),
                if (isLive) SizedBox(width: gap),
                if (isLive)
                  Container(
                    padding: EdgeInsets.symmetric(
                      horizontal: pillHP,
                      vertical: pillVP,
                    ),
                    decoration: BoxDecoration(
                      color: colors.fg.withOpacity(.06),
                      borderRadius: BorderRadius.circular(999),
                      border: Border.all(color: colors.outline),
                    ),
                    child: Text(
                      'Live',
                      style: theme.textTheme.bodySmall?.copyWith(
                        color: colors.fg,
                      ),
                    ),
                  ),
              ],
            ),

            SizedBox(height: gap),

            if (head.isNotEmpty) ...[
              Text(
                head,
                softWrap: true,
                style: theme.textTheme.titleSmall?.copyWith(
                  fontWeight: FontWeight.w600,
                  color: colors.fg,
                ),
              ),
              SizedBox(height: 6.sx(context)),
            ],
            if (expl.isNotEmpty) ...[
              Text(
                expl,
                softWrap: true,
                style: theme.textTheme.bodyMedium?.copyWith(color: colors.fg),
              ),
              SizedBox(height: gap),
            ],

            if (acts.isNotEmpty)
              ...acts.map(
                (a) => Padding(
                  padding: EdgeInsets.only(bottom: 6.sx(context)),
                  child: Row(
                    crossAxisAlignment: CrossAxisAlignment.start,
                    children: [
                      Text('•  ', style: TextStyle(color: colors.fg)),
                      Expanded(
                        child: Text(
                          a,
                          softWrap: true,
                          style: theme.textTheme.bodyMedium?.copyWith(
                            color: colors.fg,
                          ),
                        ),
                      ),
                    ],
                  ),
                ),
              )
            else
              Text(
                _motivationalFallback(msg),
                softWrap: true,
                style: theme.textTheme.bodyMedium?.copyWith(color: colors.fg),
              ),

            if (why.isNotEmpty) ...[
              SizedBox(height: gap),
              Text(
                'Why now',
                style: theme.textTheme.bodySmall?.copyWith(
                  fontWeight: FontWeight.w700,
                  color: colors.fg,
                ),
              ),
              SizedBox(height: 6.sx(context)),
              ...why
                  .take(3)
                  .map(
                    (s) => Padding(
                      padding: EdgeInsets.only(bottom: 6.sx(context)),
                      child: Row(
                        crossAxisAlignment: CrossAxisAlignment.start,
                        children: [
                          Text('•  ', style: TextStyle(color: colors.fg)),
                          Expanded(
                            child: Text(
                              s,
                              softWrap: true,
                              style: theme.textTheme.bodyMedium?.copyWith(
                                color: colors.fg,
                              ),
                            ),
                          ),
                        ],
                      ),
                    ),
                  ),
            ],

            if (showChips && (city.isNotEmpty || tags.isNotEmpty)) ...[
              SizedBox(height: gap),
              Wrap(
                children: [
                  if (city.isNotEmpty)
                    _chip(
                      context,
                      Icons.location_on_outlined,
                      city,
                      colors.fg,
                      colors.outline,
                    ),
                  ...tags.map(
                    (t) => _chip(
                      context,
                      Icons.sell_outlined,
                      t,
                      colors.fg,
                      colors.outline,
                    ),
                  ),
                ],
              ),
            ],

            if (showMetrics &&
                (hr != null ||
                    spo2 != null ||
                    skin != null ||
                    co2 != null ||
                    amb != null ||
                    aqi != null ||
                    uv != null)) ...[
              SizedBox(height: gap),
              Theme(
                data: theme.copyWith(dividerColor: Colors.transparent),
                child: ExpansionTile(
                  dense: true,
                  tilePadding: EdgeInsets.zero,
                  childrenPadding: EdgeInsets.zero,
                  iconColor: colors.fg,
                  collapsedIconColor: colors.fg,
                  title: Text(
                    'Metrics',
                    style: theme.textTheme.bodyMedium?.copyWith(
                      color: colors.fg,
                      fontWeight: FontWeight.w600,
                    ),
                  ),
                  children: [
                    SizedBox(height: 6.sx(context)),
                    Wrap(
                      spacing: 8.sx(context),
                      runSpacing: 8.sx(context),
                      children: [
                        if (hr != null)
                          _chip(
                            context,
                            Icons.favorite_border,
                            'HR ${hr.toInt()} bpm',
                            colors.fg,
                            colors.outline,
                          ),
                        if (spo2 != null)
                          _chip(
                            context,
                            Icons.bloodtype_outlined,
                            'SpO₂ ${spo2.toInt()} %',
                            colors.fg,
                            colors.outline,
                          ),
                        if (skin != null)
                          _chip(
                            context,
                            Icons.thermostat,
                            'Skin ${skin.toStringAsFixed(1)} °C',
                            colors.fg,
                            colors.outline,
                          ),
                        if (co2 != null)
                          _chip(
                            context,
                            Icons.air,
                            'CO₂ ${co2.toInt()} ppm',
                            colors.fg,
                            colors.outline,
                          ),
                        if (amb != null)
                          _chip(
                            context,
                            Icons.device_thermostat,
                            'Ambient ${amb.toStringAsFixed(1)} °C',
                            colors.fg,
                            colors.outline,
                          ),
                        if (aqi != null)
                          _chip(
                            context,
                            Icons.cloud,
                            'AQI ${aqi.toInt()}',
                            colors.fg,
                            colors.outline,
                          ),
                        if (uv != null)
                          _chip(
                            context,
                            Icons.wb_sunny_outlined,
                            'UV ${uv.toString()}',
                            colors.fg,
                            colors.outline,
                          ),
                      ],
                    ),
                  ],
                ),
              ),
            ],
          ],
        ),
      ),
    );
  }
}
