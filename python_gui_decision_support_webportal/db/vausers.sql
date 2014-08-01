create user 'elena'@'localhost' identified by 'elena';
create user 'barbara'@'localhost' identified by 'barbara';
create user 'crystal'@'localhost' identified by 'crystal';
grant all privileges on va_pupc.* to 'elena'@'localhost';
grant all privileges on va_pupc.* to 'barbara'@'localhost';
grant all privileges on va_pupc.* to 'crystal'@'localhost';
