import math

from hl_ku_core.nmea import Gga, Hdt, Rmc, nmea_checksum, parse_sentence


def sentence(payload: str) -> str:
    return f"${payload}*{nmea_checksum(payload):02X}"


def test_parse_um982_gn_sentences():
    gga = parse_sentence(
        sentence("GNGGA,123519.00,3723.2475,N,12701.1234,E,4,25,0.7,51.2,M,19.0,M,0.8,42")
    )
    assert isinstance(gga, Gga)
    assert gga.fix_quality == 4
    assert gga.satellites == 25
    assert math.isclose(gga.correction_age_sec, 0.8)

    rmc = parse_sentence(
        sentence("GNRMC,123520.00,A,3723.2475,N,12701.1234,E,1.0,90.0,310826,,,A")
    )
    assert isinstance(rmc, Rmc)
    assert math.isclose(rmc.ground_speed_mps, 0.5144444444444445)


def test_parse_um982_ths_validity():
    heading = parse_sentence(sentence("GNTHS,341.3344,A"))
    assert isinstance(heading, Hdt)
    assert heading.valid
    assert math.isclose(heading.heading_true_deg, 341.3344)

    invalid = parse_sentence(sentence("GNTHS,,V"))
    assert isinstance(invalid, Hdt)
    assert not invalid.valid
    assert math.isnan(invalid.heading_true_deg)


def test_bad_checksum_is_rejected():
    assert parse_sentence("$GNTHS,90.0,A*00") is None


def test_invalid_coordinate_range_and_hemisphere_are_not_accepted():
    invalid_latitude = parse_sentence(
        sentence("GNGGA,123519.00,9160.0000,N,12701.1234,E,4,25,0.7,51.2,M,19.0,M,0.8,42")
    )
    invalid_hemisphere = parse_sentence(
        sentence("GNGGA,123519.00,3723.2475,X,12701.1234,E,4,25,0.7,51.2,M,19.0,M,0.8,42")
    )
    assert isinstance(invalid_latitude, Gga) and math.isnan(invalid_latitude.latitude_deg)
    assert isinstance(invalid_hemisphere, Gga) and math.isnan(invalid_hemisphere.latitude_deg)
