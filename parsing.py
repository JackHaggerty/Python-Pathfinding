import re
import click
from maps import Location, Map, read_map

LOCATION_REGEX=r'(\d+),(\d+)'
def parse_location(location_str:str) -> Location:
    match = re.match(LOCATION_REGEX, location_str)
    assert match is not None
    return int(match.group(1)),int(match.group(2))


def validate_location(_ctx, _param, location_arg) -> tuple:
    try:
        return parse_location(location_arg)
    except:
        raise click.BadParameter(f"Location '{location_arg}' is invalid")

def validate_map(_ctx, _param, file_path) -> Map:
    return read_map(file_path)

